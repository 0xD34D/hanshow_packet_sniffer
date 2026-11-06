/*
 * printf_uart.c -- minimal printf() targeting this board's hardware UART.
 */

#include "driver.h"
#include <stdarg.h>

#ifdef printf
#undef printf
#endif
#include "printf_uart.h"

#define UART_TX_DMA_MAX 512

__attribute__((aligned(4))) static unsigned char uart_tx_buf[2][4 + UART_TX_DMA_MAX];
static unsigned char uart_tx_cur = 0;
static unsigned int uart_tx_pos = 0;

static void putc_uart(char c)
{
    if (c == '\n' && uart_tx_pos < UART_TX_DMA_MAX) {
        uart_tx_buf[uart_tx_cur][4 + uart_tx_pos++] = '\r';
    }
    if (uart_tx_pos < UART_TX_DMA_MAX) {
        uart_tx_buf[uart_tx_cur][4 + uart_tx_pos++] = (unsigned char)c;
    }
}

static int print_uint(unsigned int val, unsigned int base, int uppercase, int width, int zero_pad)
{
    static const char digits_lower[] = "0123456789abcdef";
    static const char digits_upper[] = "0123456789ABCDEF";
    const char *digits = uppercase ? digits_upper : digits_lower;
    char buf[12];
    int len = 0;

    if (val == 0) {
        buf[len++] = '0';
    } else {
        while (val > 0) {
            buf[len++] = digits[val % base];
            val /= base;
        }
    }

    int written = (width > len) ? width : len;
    for (int pad = len; pad < width; pad++) {
        putc_uart(zero_pad ? '0' : ' ');
    }
    while (len > 0) {
        putc_uart(buf[--len]);
    }
    return written;
}

static int print_int(int val, int width, int zero_pad)
{
    if (val < 0) {
        putc_uart('-');
        return 1 + print_uint((unsigned int)(-val), 10, 0, width, zero_pad);
    }
    return print_uint((unsigned int)val, 10, 0, width, zero_pad);
}

int printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int count = 0;

    /* This slot was last used two calls ago (ping-pong) -- wait only if
     * that transfer is somehow still running. */
    while (!(reg_uart_status1 & FLD_UART_TX_DONE));
    uart_tx_pos = 0;

    for (const char *p = fmt; *p; p++) {
        if (*p != '%') {
            putc_uart(*p);
            count++;
            continue;
        }

        p++;
        int zero_pad = 0;
        int width = 0;

        if (*p == '0') {
            zero_pad = 1;
            p++;
        }
        while (*p >= '0' && *p <= '9') {
            width = width * 10 + (*p - '0');
            p++;
        }

        switch (*p) {
            case 'c':
                putc_uart((char)va_arg(args, int));
                count++;
                break;
            case 's': {
                const char *s = va_arg(args, const char *);
                if (!s) s = "(null)";
                while (*s) { putc_uart(*s++); count++; }
                break;
            }
            case 'd':
                count += print_int(va_arg(args, int), width, zero_pad);
                break;
            case 'u':
                count += print_uint(va_arg(args, unsigned int), 10, 0, width, zero_pad);
                break;
            case 'x':
                count += print_uint(va_arg(args, unsigned int), 16, 0, width, zero_pad);
                break;
            case 'X':
                count += print_uint(va_arg(args, unsigned int), 16, 1, width, zero_pad);
                break;
            case '%':
                putc_uart('%');
                count++;
                break;
            case '\0':
                goto done;
            default:
                putc_uart('%');
                putc_uart(*p);
                count += 2;
                break;
        }
    }

done:
    va_end(args);

    unsigned char *buf = uart_tx_buf[uart_tx_cur];
    buf[0] = (unsigned char)uart_tx_pos;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    uart_send_dma(buf);
    uart_tx_cur ^= 1;

    return count;
}
