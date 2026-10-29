# fpga_link

SPI register link between a **Nucleo-G431KB** and a **Basys 3**: the STM32 reads
and writes FPGA registers over SPI with a CRC on every frame, plus an FPGA loop
tick and an interrupt line. This is the FPGA side, the STM32 firmware, the tests
and the scripts to bring it up, in one folder.

## Quick start

1. **Bitstream:** `vivado -mode batch -nojournal -nolog -source scripts/build_bitstream.tcl`,
   then `... -source scripts/program_fpga.tcl`. The Basys display reads `0000`.
2. **Firmware:** drag `prebuilt/FPGA_Link.bin` onto the **NOD_G431KB** drive,
   or run `python scripts/flash_stm32.py`.
3. **Check:** `python scripts/serial_monitor.py --reset` (needs `pip install pyserial`),
   or open the Nucleo's COM port at 115200 in any terminal and press reset.

Expected: `Init: OK`, a clean 10,000-pass self-test, then one status line per
second with ticks climbing ~1000/s and zero CRC errors. Basys LEDs follow the
switches; btnC re-runs the self-test; the Nucleo's LD2 blinks slowly (fast =
FPGA not found, solid = init error).

## Wiring (Pmod JA ↔ Nucleo-32)

| Signal | Nucleo | STM32 | Basys 3 | Artix-7 |
|---|---|---|---|---|
| SPI_SCK | D13 | PB3 | JA1 | J1 |
| SPI_MOSI | D11 | PB5 | JA2 | L2 |
| SPI_MISO | D12 | PB4 | JA3 | J2 |
| SPI_CS_N | D10 | PA11 | JA4 | G2 |
| GND | GND (CN4-4) | — | JA5 | — |
| FPGA_IRQ | D2 | PA12 | JA7 | H1 |
| LOOP_TICK | D3 | PB0 | JA8 | K2 |
| FPGA_RST_N | D6 | PB6 | JA9 | H2 |
| FAULT_N | D9 | PA8 | JA10 | G3 |
| GND | GND (CN3-2) | — | JA11 | — |

JA6 and JA12 (VCC) stay unconnected; each board runs from its own USB.

## Folder

```
rtl/ constraints/ sim/      FPGA design, Basys 3 pins, testbenches
firmware/FPGA_Link/         STM32CubeIDE project (File > Import > Existing Projects)
prebuilt/                   FPGA_Link.bin/.hex/.elf built from firmware/
scripts/                    Vivado build/program, STM32 flash, serial check
test/                       builds the STM32 driver on a PC for the replay test
Makefile                    make test / make firmware
```

## Register map (version 1)

| Addr | Name | Access | Description |
|---|---|---|---|
| 0x00 | ID | RO | 0x47344650 ("G4FP") |
| 0x01 | VERSION | RO | 1 |
| 0x02 | SCRATCH | RW | Link test register |
| 0x03 | LEDS | RW | [15:0] LEDs |
| 0x04 | SWITCHES | RO | [15:0] switches, [20:16] buttons U, L, R, D, C (debounced) |
| 0x05 | SEG7 | RW | [15:0] hex digits, [19:16] digit enables, [20] use register (else frame count) |
| 0x06 | TICK_DIV | RW | LOOP_TICK period in 100 MHz clocks; 0 = off; 100000 = 1 kHz |
| 0x07 | TICK_COUNT | RO | Ticks since reset |
| 0x08 | IRQ_STATUS | RW1C | [0] tick, [1] switch/button change, [2] CRC error |
| 0x09 | IRQ_ENABLE | RW | Same bits; FPGA_IRQ = any enabled status bit |
| 0x0A | FRAMES_OK | RO | Good frames |
| 0x0B | CRC_ERRS | RO | Rejected frames |
| 0x0C | TIME_US | RO | Free-running microsecond timestamp |
| 0x0D | CTRL | W | [0] clear FRAMES_OK, CRC_ERRS and TICK_COUNT |

## Protocol

- SPI mode 0 (CPOL 0, CPHA 0), MSB first, CS is a GPIO driven by the MCU.
  Tested at 5.3 and 10.6 MHz.
- CRC-8/SMBUS: poly 0x07, init 0x00, no reflection, no final XOR; check value
  for "123456789" is 0xF4.
- Write, 6 bytes: `[0|addr7] [D31..24] [D23..16] [D15..8] [D7..0] [CRC(0..4)]`.
  Applied only if the CRC matches; otherwise dropped, counted in CRC_ERRS,
  and FAULT_N pulses low.
- Read, 7 bytes: MOSI `[1|addr7] [CRC(byte0)] 00 00 00 00 00`; MISO data in
  bytes 2–5, CRC over the data in byte 6. A bad header CRC returns FFFFFFFF
  with an inverted CRC.
- Frames cut short by releasing CS are discarded.
- Unmapped reads return 0xDEADBEEF. Incompatible changes bump VERSION, and
  the driver's `fpga_init` refuses a mismatch.
