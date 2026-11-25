#include "driver.h"
#include <string.h>
#include "printf_uart.h"

#ifndef _attribute_ram_code_
#define _attribute_ram_code_ _attribute_ram_code_sec_
#endif

#ifndef rf_rx_finish
#define rf_rx_finish rf_is_rx_finish
#endif

#define rf_rx_buffer_reconfig(buf) rf_rx_buffer_reconfig((unsigned short)(uint32_t)(buf))

/* =========================================================================
 * Hardware Pinout (Nebular Tag / TLSR8258)
 * ========================================================================= */
#define LED_RED     GPIO_PB4
#define LED_GREEN   GPIO_PD3
#define UART_TXD    GPIO_PB1
#define UART_RXD    GPIO_PA0

static inline void led_red_on(void)     { gpio_write(LED_RED, 0); }
static inline void led_red_off(void)    { gpio_write(LED_RED, 1); }
static inline void led_green_on(void)   { gpio_write(LED_GREEN, 0); }
static inline void led_green_off(void)  { gpio_write(LED_GREEN, 1); }

static void led_init(void) {
    gpio_set_func(LED_RED, AS_GPIO);
    gpio_set_output_en(LED_RED, 1);
    gpio_set_input_en(LED_RED, 0);
    led_red_off();

    gpio_set_func(LED_GREEN, AS_GPIO);
    gpio_set_output_en(LED_GREEN, 1);
    gpio_set_input_en(LED_GREEN, 0);
    led_green_off();
}

/* =========================================================================
 * UART Low-Level Functions (921600 8N1 on PB1 TX, PA0 RX)
 * ========================================================================= */
static void uart_init_sniffer(void) {
    gpio_set_func(UART_TXD, AS_UART);
    gpio_set_func(UART_RXD, AS_GPIO);
    gpio_set_input_en(UART_RXD, 1);
    gpio_set_output_en(UART_RXD, 0);

    uart_gpio_set(UART_TX_PB1, UART_RX_PA0);

    /* 921600 baud @ 24MHz sys_clk: (div+1)*(bwpc+1) = 2*13 = 26 -> 923077 baud (+0.16%) */
    uart_reset();
    uart_init(1, 12, PARITY_NONE, STOP_BIT_ONE);

    uart_dma_enable(0, 1); /* TX via DMA (see printf_uart.c); RX stays polled (uart_ndma_read_byte()) */
    irq_disable_type(FLD_IRQ_UART_EN);

    sleep_us(10000);
}

/* =========================================================================
 * Radio Configuration — Native 5B Check-In Correlator
 * ========================================================================= */
#define RF_RX_BUF_SIZE 256
static uint8_t rf_rx_buf[RF_RX_BUF_SIZE] __attribute__((aligned(4)));

/* Two listen modes, toggled live with 'm':
 *   MODE_TAG    - 5-byte native correlator, 180-byte 5x-repetition tag check-in frames.
 *   MODE_BEACON - 4-byte store-beacon correlator, direct (non-5x) AP beacon frames. */
typedef enum { MODE_TAG = 0, MODE_BEACON = 1 } sniff_mode_t;

static const uint8_t acc_native_sync[5] = {0xF8, 0x3E, 0x0F, 0x83, 0xE0}; /* 5B native checkin */
static const uint8_t acc_beacon_sync[4] = {0xB1, 0x36, 0x7C, 0xCA};       /* 4B AP store beacon */

/* Max accepted length for a direct beacon frame (max-length filter in reg 0x406, NOT a fixed
 * length — shorter frames still complete). Store beacons are 17B (0x11) or 20B (0x14), but this
 * is a packet sniffer: keep generous headroom so anything on this sync word is captured, not
 * filtered. The dma_len gate below is deliberately wide for the same reason. */
#define BEACON_MAX_LEN 64

static sniff_mode_t current_mode     = MODE_TAG; /* 'm' toggles; boot in tag mode (prior behavior) */
static int16_t  current_500k_chn = 2; /* 2401.0 MHz */
static uint32_t packet_count     = 0;
static uint32_t last_rx_tick     = 0;
static bool     verbose_output   = true;  /* 'v' toggles. Off => raw HEX line per packet only. */

static void set_rx_hardware(void) {
    rf_set_tx_rx_off();
    rf_trx_state_set(RF_MODE_RX, 0);
    rf_set_channel_500k(current_500k_chn, 0);

    /* ShockBurst 500 kbps Mode 3 */
    write_reg8(0x404, (read_reg8(0x404) & ~0x07) | 0x03);

    if (current_mode == MODE_TAG) {
        /* 5-byte native sync word, fixed 180-byte 5x-repetition check-in frame */
        rf_acc_len_set(5);
        write_reg8(0x405, 5);
        write_reg8(0x420, 5 * 8);
        for (uint8_t i = 0; i < 5; i++) {
            write_reg8(0x408 + i, acc_native_sync[i]);
        }
        write_reg8(0x407, 0x01); /* Enable Pipe 0 only */
        write_reg8(0x406, 180);  /* 180-byte 5x-repetition check-in frame */
    } else {
        /* 4-byte store-beacon sync word, short direct (non-5x) beacon frame */
        rf_acc_len_set(4);
        write_reg8(0x405, 4);
        write_reg8(0x420, 4 * 8);
        for (uint8_t i = 0; i < 4; i++) {
            write_reg8(0x408 + i, acc_beacon_sync[i]);
        }
        write_reg8(0x407, 0x01);           /* Enable Pipe 0 only */
        write_reg8(0x406, BEACON_MAX_LEN);  /* max-length filter for the short beacon frame */
    }

    rf_rx_buf[0] = 0;
    rf_rx_buf[1] = 0;
    rf_rx_buffer_reconfig(rf_rx_buf);
    rf_set_rxmode();
}

static void radio_init_sniffer(void) {
    reset_baseband();
    rf_drv_init(RF_MODE_PRIVATE_500K);
    rf_set_preamble_len(4);
    rf_set_power_level_index(RF_POWER_P10p46dBm);

    rf_rx_buffer_set(rf_rx_buf, RF_RX_BUF_SIZE, 0);
    set_rx_hardware();
}

/* =========================================================================
 * 5x-Repetition Majority-Voting Decoder
 * ========================================================================= */
static void decode_5x_repetition_range(const uint8_t *in_bytes, uint8_t *out_bytes, int start_byte, int count_bytes) {
    int start_bit = start_byte * 8;
    int end_bit = (start_byte + count_bytes) * 8;
    for (int b = start_byte; b < start_byte + count_bytes; b++) {
        out_bytes[b] = 0;
    }
    for (int out_bit = start_bit; out_bit < end_bit; out_bit++) {
        int ones = 0;
        for (int k = 0; k < 5; k++) {
            int idx = out_bit * 5 + k;
            uint8_t b = in_bytes[idx >> 3];
            if ((b >> (7 - (idx & 7))) & 1) ones++;
        }
        if (ones >= 3) {
            out_bytes[out_bit >> 3] |= (1 << (7 - (out_bit & 7)));
        }
    }
}

/* =========================================================================
 * Packet Processing & Diagnostic Logging
 * ========================================================================= */
static void process_tag_packet(uint32_t rx_tick) {
    uint32_t dma_len = rf_rx_buf[0] | (rf_rx_buf[1] << 8);
    if (dma_len < 180 || dma_len > 220) {
        /* Native 5B check-in frames are 180 bytes; anything else is noise on this sync word. */
        return;
    }

    /* Snapshot current DMA buffer so radio can immediately re-arm without collision */
    static uint8_t pkt_copy[RF_RX_BUF_SIZE];
    memcpy(pkt_copy, rf_rx_buf, (dma_len + 6 > RF_RX_BUF_SIZE) ? RF_RX_BUF_SIZE : (dma_len + 6));

    /* Decode the full 32-byte frame from the 5x-repetition payload */
    uint8_t decoded_buf[32];
    decode_5x_repetition_range(&pkt_copy[24], decoded_buf, 0, 32);

    uint8_t *payload = &decoded_buf[4];
    uint8_t payload_len = 26;

    /* Re-arm RX buffer immediately */
    rf_rx_buf[0] = 0;
    rf_rx_buf[1] = 0;
    rf_rx_buffer_reconfig(rf_rx_buf);
    rf_set_rxmode();

    uint8_t ftype = payload[0];

    /* Turnaround / Delta T Calculation */
    uint32_t delta_ticks = rx_tick - last_rx_tick;
    uint32_t delta_us    = delta_ticks / 16;
    last_rx_tick         = rx_tick;

    /* Verbose mode: full decoded output. Non-verbose: only the raw HEX line per packet
     * (for noisy environments with hundreds of tags transmitting). Toggle with 'v'. */
    if (verbose_output) {
    uint8_t raw_rssi = pkt_copy[dma_len + 2];
    int8_t rssi_dbm = (int8_t)raw_rssi - 110;
    uint8_t pipe = (pkt_copy[dma_len + 3] >> 3) & 0x07;
    bool hw_crc_ok = RF_SB_PACKET_CRC_OK(pkt_copy);

    printf("\n============================================================\n");
    printf("📡 [SNIFFER] PKT #%d | ", (int)packet_count);

    /* Microsecond Delta since previous packet */
    if (packet_count > 1) {
        if (delta_us >= 1000) {
            printf("Δt=%d.%03d ms | ", (int)(delta_us / 1000), (int)(delta_us % 1000));
        } else {
            printf("Δt=%d us | ", (int)delta_us);
        }
    }

    printf("RSSI: %d dBm | Pipe: %d | CRC: %s\n", (int)rssi_dbm, (int)pipe, hw_crc_ok ? "OK" : "CRC_FAIL");
    printf("------------------------------------------------------------\n");

    /* Classification & Opcode Decoding */
    if (ftype == 0x91 || ftype == 0x8A) {
        /* 0x8A is the same check-in frame as 0x91: the tag's periodic check-in builder
         * substitutes opcode 0x8A for 0x91 whenever its internal retry-attempt counter
         * exceeds 8, then resets the counter (silicon 0x1C726-0x1C73A). Same layout. */
        if (ftype == 0x91) {
            printf("  🔼 [UPLINK] 0x91 Tag Check-In (180B 5x-repetition on Ch 2)\n");
        } else {
            printf("  🔼 [UPLINK] 0x8A Tag Check-In (retry variant: retry counter > 8)\n");
        }
        printf("     Tag ID : %02X:%02X:%02X:%02X\n", payload[1], payload[2], payload[3], payload[4]);

        /* Bytes 5-7 are the tag-identity struct slot *(0x00843324+6..8), NOT a sequence.
         * (Silicon: builder at 0x1C5DC/0x1C5E0/0x1C5E4 loads [r5,#6/#7/#8].) The 0x91 frame
         * carries NO sequence/tier field -- the sequence tier is internal tag state only.
         * byte5/6 = MAC-suffix hi/lo, byte7 = HW-profile revision. */
        printf("     MAC Suffix: %02X:%02X | HW Rev: 0x%02X\n", payload[5], payload[6], payload[7]);

        uint16_t slot_val = payload[9] | ((payload[10] & 0x0F) << 8);
        uint8_t  batt_adc = (payload[10] >> 4) & 0x0F;
        uint8_t  chn_val  = payload[11];
        uint16_t pan_val  = payload[12] | (payload[13] << 8);
        /* Model from the 32-bit HW descriptor at bytes 20-23 (uncorrupted by commission). */
        uint32_t hw_prof = (uint32_t)payload[20] | ((uint32_t)payload[21] << 8) | ((uint32_t)payload[22] << 16) | ((uint32_t)payload[23] << 24);
        const char *model_str = (hw_prof == 0x0C610D49) ? "1.54\" BWRY" :
                                (hw_prof == 0x0C860DFE || hw_prof == 0x0C5B0D3D) ? "2.66\" BWRY" : "Nebular ESL";
        printf("     Model: %s (HW %02X %02X %02X %02X) | Slot: %d | Batt: %d/15 | Ch: %d | PAN: 0x%04X\n",
               model_str, payload[20], payload[21], payload[22], payload[23],
               slot_val, batt_adc, chn_val, pan_val);
    } else if (ftype == 0x94) {
        printf("  🔼 [UPLINK] 0x94 Tag Fast-Poll Wakeup / Status ACK\n");
        printf("     Tag ID: %02X:%02X:%02X:%02X | Seq: 0x%02X\n",
               payload[1], payload[2], payload[3], payload[4], payload[5]);
    } else if (ftype == 0x97) {
        printf("  🔼 [UPLINK] 0x97 Session Setup\n");
        printf("     Tag ID: %02X:%02X:%02X:%02X | Seq: 0x%02X\n",
               payload[1], payload[2], payload[3], payload[4], payload[5]);
    } else if (ftype == 0x52 || ftype == 0x51) {
        /* 0x52: tag emits a 10x burst after an NFC 0x81 Fast-Bind. 0x51: Fast-Poll
         * downlink / commission-confirm ACK. Same on-air field layout. */
        uint16_t raw_slot = payload[7] | ((payload[8] & 0x0F) << 8);
        uint8_t  raw_chn  = payload[9];
        printf("  🔼 [UPLINK] 0x%02X NFC Fast-Bind %s\n",
               ftype, (ftype == 0x52) ? "Burst" : "ACK");
        printf("     Tag ID: %02X:%02X:%02X:%02X | Seq: 0x%02X | Slot: %d | Ch: %d\n",
               payload[1], payload[2], payload[3], payload[4], payload[5], raw_slot, raw_chn);
    } else {
        printf("  📦 [FRAME] Raw Opcode: 0x%02X (Len %dB)\n", ftype, payload_len);
    }
    } /* end verbose_output */

    /* Raw Hex Dump of the decoded 26-byte frame (printed in both modes) */
    printf(verbose_output ? "     HEX: " : "HEX: ");
    for (uint8_t i = 0; i < payload_len && i < 220; i += 16) {
        /* One printf() per 16 bytes (not per byte) so each DMA transfer carries a useful amount of data */
        char hexline[16 * 3 + 1];
        int n = 0;
        for (uint8_t j = i; j < payload_len && j < 220 && j < i + 16; j++) {
            hexline[n++] = "0123456789ABCDEF"[payload[j] >> 4];
            hexline[n++] = "0123456789ABCDEF"[payload[j] & 0x0F];
            hexline[n++] = ' ';
        }
        hexline[n] = '\0';
        printf("%s", hexline);
    }
    printf("\n");
}

/* =========================================================================
 * AP Store-Beacon Processing (direct, non-5x; 4-byte correlator B1 36 7C CA)
 * ========================================================================= */
static void process_beacon_packet(uint32_t rx_tick) {
    uint32_t dma_len = rf_rx_buf[0] | (rf_rx_buf[1] << 8);
    if (dma_len < 4 || dma_len > BEACON_MAX_LEN) {
        /* Wide gate on purpose: reject only empty/overlong DMA artifacts, capture everything
         * else on this sync word (both 17B/20B beacon variants and any other short frame). */
        return;
    }

    /* Snapshot the DMA buffer so the radio can re-arm without colliding with the decode. */
    static uint8_t pkt_copy[RF_RX_BUF_SIZE];
    uint32_t copy_len = (dma_len + 6 > RF_RX_BUF_SIZE) ? RF_RX_BUF_SIZE : (dma_len + 6);
    memcpy(pkt_copy, rf_rx_buf, copy_len);

    /* Re-arm RX buffer immediately */
    rf_rx_buf[0] = 0;
    rf_rx_buf[1] = 0;
    rf_rx_buffer_reconfig(rf_rx_buf);
    rf_set_rxmode();

    /* Direct frame: the DMA writes the actual length to pkt_copy[0..3], so the on-air
     * payload begins at pkt_copy[4]. (Offset verified against the tag path's dma_len/RSSI
     * handling; confirm on the first live beacon capture via the raw HEX line below.) */
    const uint8_t *frame = &pkt_copy[4];

    /* Byte 0 is the in-band frame length: 0x11 = 17B beacon, 0x14 = 20B beacon (count includes
     * byte 0 through the trailing CRC). When it's a known beacon length we actually captured,
     * trust it and show exactly that many bytes; otherwise fall back to the raw DMA length so
     * unknown/other frames on this sync word are still shown in full. */
    bool     known_beacon = (frame[0] == 0x11 || frame[0] == 0x14);
    uint32_t frame_len    = (known_beacon && frame[0] <= dma_len) ? frame[0] : dma_len;

    /* Turnaround / Delta T since previous captured frame */
    uint32_t delta_ticks = rx_tick - last_rx_tick;
    uint32_t delta_us    = delta_ticks / 16;
    last_rx_tick         = rx_tick;

    if (verbose_output) {
        uint8_t raw_rssi   = pkt_copy[dma_len + 2];
        int8_t  rssi_dbm   = (int8_t)raw_rssi - 110;
        bool    hw_crc_ok  = RF_SB_PACKET_CRC_OK(pkt_copy);

        printf("\n============================================================\n");
        printf("📡 [BEACON] PKT #%d | ", (int)packet_count);
        if (packet_count > 1) {
            if (delta_us >= 1000) {
                printf("Δt=%d.%03d ms | ", (int)(delta_us / 1000), (int)(delta_us % 1000));
            } else {
                printf("Δt=%d us | ", (int)delta_us);
            }
        }
        printf("RSSI: %d dBm | CRC: %s\n", (int)rssi_dbm, hw_crc_ok ? "OK" : "CRC_FAIL");
        printf("------------------------------------------------------------\n");

        /* Documented store-beacon layout (CLAUDE.md): byte0=len (0x11/0x14), 1=PAN lo,
         * 2=epoch/seq, 3=PAN hi, 4=channel, 5..6=schedule offset, 7=target slot,
         * 9=flags (0x01=update scheduled), 10=cadence tier, 11..14=page bitmask. */
        if (known_beacon) {
            uint16_t pan_val   = frame[1] | (frame[3] << 8);
            uint16_t sched_off = frame[5] | (frame[6] << 8);
            printf("  🏪 [STORE BEACON] %s (%dB) | PAN: 0x%04X | Ch: %d | Seq: 0x%02X\n",
                   (frame[0] == 0x11) ? "short" : "extended", frame[0],
                   pan_val, frame[4], frame[2]);
            printf("     Target Slot: %d | Flags: 0x%02X%s | Cadence: 0x%02X | Sched: %d\n",
                   frame[7], frame[9], (frame[9] & 0x01) ? " (update scheduled)" : "",
                   frame[10], sched_off);
            printf("     Page Bitmask: %02X %02X %02X %02X\n",
                   frame[11], frame[12], frame[13], frame[14]);
        } else {
            printf("  📦 [FRAME] Beacon sync, unexpected lead byte 0x%02X (Len %dB)\n",
                   frame[0], (int)frame_len);
        }
    }

    /* Raw Hex Dump of the direct beacon frame (printed in both modes) */
    printf(verbose_output ? "     HEX: " : "HEX: ");
    for (uint32_t i = 0; i < frame_len; i += 16) {
        char hexline[16 * 3 + 1];
        int n = 0;
        for (uint32_t j = i; j < frame_len && j < i + 16; j++) {
            hexline[n++] = "0123456789ABCDEF"[frame[j] >> 4];
            hexline[n++] = "0123456789ABCDEF"[frame[j] & 0x0F];
            hexline[n++] = ' ';
        }
        hexline[n] = '\0';
        printf("%s", hexline);
    }
    printf("\n");
}

/* =========================================================================
 * Packet Dispatcher — routes to the active listen mode
 * ========================================================================= */
static void process_packet(uint32_t rx_tick) {
    packet_count++;
    led_green_on();
    if (current_mode == MODE_TAG) {
        process_tag_packet(rx_tick);
    } else {
        process_beacon_packet(rx_tick);
    }
    led_green_off();
}

/* =========================================================================
 * User Interaction & Help
 * ========================================================================= */
static void print_banner(void) {
    printf("\n");
    printf("============================================================\n");
    printf("  🛰️  HANSHOW NEBULAR DEDICATED GFSK RF PACKET SNIFFER\n");
    printf("  Channel  : 2 (2401.0 MHz, 500 kbps GFSK ShockBurst Mode 3)\n");
    printf("============================================================\n");
    printf(" Mode       : %s\n",
           current_mode == MODE_TAG
               ? "TAG  (5B sync F8 3E 0F 83 E0, 180B 5x-repetition check-ins)"
               : "BEACON (4B sync B1 36 7C CA, direct AP store beacons)");
    printf(" Verbose    : %s  ('v' toggles; off = raw HEX line per packet only)\n",
           verbose_output ? "ON" : "OFF");
    printf(" Commands   : [m] Toggle Mode  [v] Toggle Verbose  [c] Clear Stats  [?] Help\n");
    printf("============================================================\n\n");
}

#define CMD_BUF_LEN 32
static char cmd_buf[CMD_BUF_LEN];
static uint8_t cmd_idx = 0;

static void poll_uart_rx(void) {
    uint8_t count = reg_uart_buf_cnt & FLD_UART_RX_BUF_CNT;
    while (count--) {
        uint8_t byte = uart_ndma_read_byte();
        if (byte == '\r' || byte == '\n') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                char cmd = cmd_buf[0];

                if (cmd == '?' || cmd == 'h') {
                    print_banner();
                } else if (cmd == 'm') {
                    current_mode = (current_mode == MODE_TAG) ? MODE_BEACON : MODE_TAG;
                    set_rx_hardware();        /* re-arm radio with the new correlator/length */
                    last_rx_tick = clock_time(); /* avoid a bogus Δt on the first new-mode packet */
                    printf("[SNIFFER] Mode -> %s.\n",
                           current_mode == MODE_TAG
                               ? "TAG (F8 3E 0F 83 E0, 5x-repetition check-ins)"
                               : "BEACON (B1 36 7C CA, direct AP store beacons)");
                } else if (cmd == 'v') {
                    verbose_output = !verbose_output;
                    printf("[SNIFFER] Verbose output %s.\n", verbose_output ? "ON (full decode)" : "OFF (raw HEX only)");
                } else if (cmd == 'c') {
                    packet_count = 0;
                    printf("[SNIFFER] Packet counter reset to 0.\n");
                }
                cmd_idx = 0;
            }
        } else if (cmd_idx < (CMD_BUF_LEN - 1) && byte >= 0x20) {
            cmd_buf[cmd_idx++] = (char)byte;
        }
    }
}

_attribute_ram_code_ void irq_handler(void) {}

/* =========================================================================
 * Main Program Loop
 * ========================================================================= */
int main(void) {
    cpu_wakeup_init();
    clock_init(SYS_CLK_24M_Crystal);
    led_init();
    uart_init_sniffer();

    /* Sign of life: Red LED on during initialization */
    led_red_on();
    sleep_us(50000);
    led_red_off();

    radio_init_sniffer();

    print_banner();

    uint32_t last_heartbeat = clock_time();

    while (1) {
        wd_clear();
        poll_uart_rx();

        /* Check for received RF packet */
        if (rf_rx_finish()) {
            uint32_t rx_tick = clock_time();
            rf_rx_finish_clear_flag();
            process_packet(rx_tick);

            /* Only re-arm if another packet did not arrive during processing */
            if (!rf_rx_finish()) {
                rf_rx_buf[0] = 0;
                rf_rx_buf[1] = 0;
                rf_rx_buffer_reconfig(rf_rx_buf);
            }
        }

        /* Periodic heartbeat blink (every 5 seconds) */
        if (clock_time_exceed(last_heartbeat, 5000000)) {
            last_heartbeat = clock_time();
            led_green_on();
            sleep_us(10000);
            led_green_off();
        }
    }

    return 0;
}
