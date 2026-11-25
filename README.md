# Hanshow Nebular GFSK RF Packet Sniffer

A dedicated, passive over-the-air (OTA) RF packet sniffer running on the **Telink TLSR8258** SoC (Nebular ESL tag).

Designed to continuously listen on **Channel 2 (2401.0 MHz, 500 kbps GFSK ShockBurst Mode 3)** for the native 180-byte 5x-repetition frames transmitted by Hanshow tags, decode them, and log them.

---

## Features

- **Pure Passive RF Listening**: RF TX circuitry and routines are 100% disabled. The sniffer will never transmit or interfere with OTA communications.
- **Two Listen Modes (`m` toggles)**: Switch live between the two over-the-air frame families on Channel 2:
  - **TAG mode** (default on boot): the 5-byte sync word `F8 3E 0F 83 E0` (bit-reversed `1F 7C F0 C1 07`), carrying the 180-byte 5x-repeated tag check-in frames. Decoded into a clean 26-byte frame via majority voting.
  - **BEACON mode**: the 4-byte sync word `B1 36 7C CA`, carrying the AP/store **beacon** frames. These are short, **direct** (non-5x) frames; the sniffer reads them straight out of the DMA buffer and decodes the documented 17-byte (`0x11`) / 20-byte (`0x14`) layout (length, PAN ID, channel, target slot, update flags, cadence tier, active-page bitmask). A raw hex line is always printed so the exact on-air bytes are visible.

  Toggling `m` re-arms the radio with the new correlator and frame length; the mode is shown in the banner (`?`).
- **Opcode Decoding**: Classifies decoded frame by opcode:
  - `0x91` - Tag Check-In (telemetry: MAC-suffix + HW revision, model, slot, battery, channel, PAN).
  - `0x8A` - Tag Check-In retry variant (tag substitutes this for `0x91` when its retry counter exceeds 8; same layout)
  - `0x94` - Fast-Poll Wakeup / Status ACK
  - `0x97` - Session Setup
  - `0x51` / `0x52` - Fast-Bind ACK / Burst
  - **any other opcode** - printed as a raw opcode with a full hex dump of the decoded frame
- **Microsecond Inter-Packet Timing**: System timer ticks at 16 MHz ($1\ \mu\text{s} = 16\text{ ticks}$). Logs the delta $\Delta t$ between consecutive captured frames.
- **Verbosity Toggle (`v`)**: Default **on** (full decoded output). When toggled **off**, each packet prints only its raw decoded hex line (e.g. `HEX: 91 FD CD E8 88 ...`) with no banner, classification, or separators — intended for noisy environments where hundreds of tags are transmitting at once.
- **UART Output & Control**: Outputs formatted logs over `PB1` TX at 921600 8N1; accepts single-key commands (`m`, `v`, `c`, `?`/`h`) over `PA0` RX.

---

## Building & Flashing

```bash
# Build binary (run from repo root)
make clean && make
```