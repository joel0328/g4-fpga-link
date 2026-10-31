# fpga_link

[![CI](https://github.com/joel0328/g4-fpga-link/actions/workflows/ci.yml/badge.svg)](https://github.com/joel0328/g4-fpga-link/actions/workflows/ci.yml)

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

Expected: `Init: OK`, clean single-frame and burst self-tests, then once a
second a ticks line (~1000/s, zero CRC errors), a control-step line (one step
per tick, none skipped) and `safety: disarmed, SW15 up, kicks on, faults 0x0`.
The board never arms by itself: put **SW15 up** and press **btnC** (re-runs
the self-tests, clears faults, arms) or type `a` in the terminal. Armed, LED15
lights and LEDs 0–13 follow the switches; **btnD is the e-stop**. The Nucleo's
LD2 blinks slowly (fast = FPGA not found, solid = init error).

## Status

Measured on hardware (Nucleo-G431KB + Basys 3, wired per the table below;
Vivado 2025.1, STM32CubeProgrammer 2.22.0). Tags mark the gates.

| Milestone | Result |
|---|---|
| **M1 link** (tag `m1-link`) | Bring-up passes at 5.3 and 10.6 MHz SPI. Self-test 10,000 write/readback pairs clean; 73.8k single register accesses/s at 10.6 MHz. 10-minute soak: 45.6M frames, 0 CRC errors. Vivado WNS 3.042 ns / WHS 0.119 ns. Switch/LED, btnC and all IRQ paths confirmed. `.ioc` regenerated in CubeMX 6.17 builds a byte-identical `.bin`. |
| **Burst frames** (map v2) | 176.8k register words/s in 16-word bursts (2.4× single frames); 1.06M frames, 0 CRC errors. Testbench 388 checks; 4 injected RTL bugs all caught. |
| **SPI DMA + 1 kHz control step** | The whole step (12-word burst read, 3-word burst write, IRQ clear) runs in interrupts: 74–89 µs from tick to done, ~17 µs CPU per step, no SPI in the main loop. 30-minute soak: 1.8M steps, 0 skipped, 0 errors. |
| **Safety watchdog** (map v3, interim, replaced by v4) | 5 ms watchdog fed by the control step. Freezing the MCU for 100 ms over SWD latched FAULT and cut the outputs; they stayed off after the MCU resumed until btnC re-armed. 30-minute soak armed: 1.8M steps, no false trips. Vivado WNS 2.033 ns. To be replaced by the full safety layer (arming, e-stop, fault causes). |
| **Safety layer** (map v4) | Arming (SW15 + key + no fault + tick), kick watchdog, latched faults WDT/LINK/ESTOP/ARM_LOST, per-tick snapshot. Testbench 494 checks incl. a per-clock armed-vs-fault invariant; 9 injected RTL bugs all caught. On hardware: stopping kicks latched WDT and cut the outputs; SW15 down → ARM_LOST; btnD → ESTOP; arming refused with SW15 down. 30-minute armed soak: 1.8M steps (5 frames each, ~112 µs), 0 skipped, 0 CRC errors, no false trips. Vivado WNS 1.905 ns. |
| **Motors** (map v5, simulation only) | Two sign-magnitude H-bridge channels (20 kHz PWM on TB6612FNG IN pins, gated by ARMED, duty clamp, duties zeroed while disarmed) and two x4 quadrature decoders with glitch filter, in the per-tick snapshot. Testbench 595 checks incl. per-clock invariants (no IN pin high while disarmed, never both IN pins of a bridge); 21 injected RTL bugs (9 safety, 12 motor/encoder) all caught. Vivado WNS 1.148 ns. Hardware gate pending: TB6612FNG + JGB37-520. |
| **Register map generator** | `regs/regmap.toml` generates the Verilog, C, Python and MATLAB constants and the table below. Regenerating map v3 reproduced all 41 existing C constants and the README table exactly; the firmware `.bin` is byte-identical. RTL now uses generated names: Vivado WNS 2.327 ns, self-tests and 1 kHz step clean on hardware. |

CI (GitHub Actions) runs the RTL testbench, the driver↔RTL replay and the
firmware build on every push.

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

## Wiring (Pmod JB ↔ TB6612FNG and encoders, map v5)

| Signal | Basys 3 | Artix-7 | Goes to |
|---|---|---|---|
| M1_IN1 | JB1 | A14 | TB6612 AIN1 |
| M1_IN2 | JB2 | A16 | TB6612 AIN2 |
| M2_IN1 | JB3 | B15 | TB6612 BIN1 |
| M2_IN2 | JB4 | B16 | TB6612 BIN2 |
| GND | JB5 / JB11 | — | TB6612 GND, encoder GND |
| 3.3 V | JB6 / JB12 | — | TB6612 VCC, STBY, PWMA, PWMB; encoder VCC |
| ENC1_A | JB7 | A15 | motor 1 encoder A |
| ENC1_B | JB8 | A17 | motor 1 encoder B |
| ENC2_A | JB9 | C15 | motor 2 encoder A |
| ENC2_B | JB10 | C16 | motor 2 encoder B |

- **Power the encoders from 3.3 V, never 5 V.** Their outputs pull up to the
  encoder supply, and the Artix-7 pins aren't 5 V tolerant.
- **Motor supply:** 12 V into the TB6612's VM, current-limited (about 1 A for
  bring-up). Tie its ground to the Basys ground. Never connect it to a Pmod pin.
- With PWMA/PWMB and STBY tied high, the FPGA drives the IN pins. Both low
  means coast, which is the state whenever the board is disarmed. While the
  FPGA is unconfigured, its pull-ups can hold both IN pins high. That is the
  TB6612's short brake, which doesn't drive the motor.

## Folder

```
regs/regmap.toml            the register map: the one source for RTL, C, Python, MATLAB, README
rtl/ constraints/ sim/      FPGA design, Basys 3 pins, testbenches
firmware/FPGA_Link/         STM32CubeIDE project (File > Import > Existing Projects)
prebuilt/                   FPGA_Link.bin/.hex/.elf built from firmware/
scripts/                    Vivado build/program, STM32 flash, serial check
test/                       builds the STM32 driver on a PC for the replay test
tools/                      generated register constants for PC-side Python and MATLAB
Makefile                    make test / make regs / make firmware
```

`make test` (Icarus Verilog + a host C compiler) runs the RTL testbench and the
driver↔RTL replay. On Windows, run it from Git Bash. CI runs it and the
firmware build on every push.

The register map is defined once, in `regs/regmap.toml`. `make regs`
(Python 3.11+, no packages) regenerates `rtl/fpga_regs.vh`, `fpga_regs.h`,
`tools/fpga_regs.py`, `tools/fpga_regs.m` and the table below; `make test`
fails if any of them is stale. The RTL uses the generated names, never
literal addresses.

<!-- regmap:begin (generated by scripts/gen_regs.py from regs/regmap.toml) -->
## Register map (version 5)

| Addr | Name | Access | Description |
|---|---|---|---|
| 0x00 | ID | RO | 0x47344650 ("G4FP") |
| 0x01 | VERSION | RO | 5 |
| 0x02 | SCRATCH | RW | Link test register |
| 0x03 | LEDS | RW | [13:0] user LEDs, lit only while armed. LED15 = armed and LED14 = fault are driven by the safety layer |
| 0x04 | SWITCHES | RO | [15:0] switches, [20:16] buttons U, L, R, D, C (debounced) |
| 0x05 | SEG7 | RW | [15:0] hex digits, [19:16] digit enables, [20] use register (else frame count) |
| 0x06 | TICK_DIV | RW | LOOP_TICK period in 100 MHz clocks; 0 = off; 100000 = 1 kHz. Writes ignored while armed |
| 0x07 | TICK_COUNT | RO | Ticks since reset |
| 0x08 | IRQ_STATUS | RW1C | [0] tick, [1] switch/button change, [2] CRC error, [3] fault latched (any cause) |
| 0x09 | IRQ_ENABLE | RW | Same bits; FPGA_IRQ = any enabled status bit |
| 0x0A | FRAMES_OK | RO | Good frames |
| 0x0B | CRC_ERRS | RO | Rejected frames |
| 0x0C | TIME_US | RO | Free-running microsecond timestamp |
| 0x0D | CTRL | W | [0] clear FRAMES_OK, CRC_ERRS, TICK_COUNT and ENC_ERRS |
| 0x10–0x1F | SCRATCH_RAM | RW | 16 free words (burst self-test) |
| 0x20 | ARM | RW | Write ARM_KEY (0x41524D21) to arm: needs SW15 up, no fault, tick running. Any other value disarms. Read: [0] armed, [1] SW15, [2] fault, [3] tick running |
| 0x21 | WDT_TICKS | RW | [7:0] watchdog limit in loop ticks (1–255, 0 is stored as 1). A tick that finds this many ticks since the last kick latches FAULT.WDT. Writes ignored while armed |
| 0x22 | FAULT_STATUS | RO | Latched causes: [0] WDT missed kicks, [1] LINK 4 rejected frames in a row, [2] ESTOP btnD, [3] ARM_LOST SW15 down while armed. Any fault disarms and holds FAULT_N low |
| 0x23 | FAULT_CLEAR | W | Write 1s to clear FAULT_STATUS bits. ESTOP stays set while btnD is held |
| 0x24 | WDT_KICK | RW | Any write feeds the watchdog. Read: [7:0] ticks since the last kick (0 while disarmed) |
| 0x25 | M1_DUTY | RW | [15:0] signed duty, ±32767 = ±100 % (−32768 acts as −32767): positive PWMs IN1, negative IN2, 0 = coast. Clamped to ±DUTY_LIMIT. Reads sign-extended. Held at 0 while disarmed (writes ignored), so arming never starts a motor |
| 0x26 | M2_DUTY | RW | Same for motor 2 |
| 0x27 | DUTY_LIMIT | RW | [14:0] largest |duty| applied to either motor; reset 16384 = 50 %. Writes ignored while armed |
| 0x28 | ENC_ERRS | RO | Illegal quadrature transitions (A and B changed together): [15:0] encoder 1, [31:16] encoder 2. Wraps; CTRL[0] clears |
| 0x30 | SNAP_TICK | RO | TICK_COUNT of the snapshot (sequence number) |
| 0x31 | SNAP_TIME_US | RO | TIME_US at the tick |
| 0x32 | SNAP_INPUTS | RO | SWITCHES at the tick |
| 0x33 | SNAP_SAFETY | RO | ARM read bits in [3:0], FAULT_STATUS in [11:8], at the tick |
| 0x34 | SNAP_ENC1_COUNT | RO | Encoder 1 position at the tick: signed 32-bit, 4 counts per encoder line, up when A leads B |
| 0x35 | SNAP_ENC1_DELTA | RO | Encoder 1 counts since the previous tick (signed) |
| 0x36 | SNAP_ENC2_COUNT | RO | Encoder 2 position at the tick |
| 0x37 | SNAP_ENC2_DELTA | RO | Encoder 2 counts since the previous tick |
| 0x7F | — | — | Reserved: the burst escape in the frame header, never a register |
<!-- regmap:end -->

## Safety layer (map v4)

The FPGA owns the actuator outputs, so a firmware bug can't run them away.
Outputs (LEDs 0–13 today, motor drives later) are forced off unless **armed**.

- **Arming** needs all of: SW15 up (hardware arm enable), the key
  `0x41524D21` written to ARM, no fault latched, and the loop tick running.
  Writing any other value to ARM disarms. TICK_DIV and WDT_TICKS can't change
  while armed.
- **Watchdog:** while armed, the MCU must write WDT_KICK; a loop tick that
  finds WDT_TICKS (default 3) ticks since the last kick latches a fault.
  Ordinary frames don't feed it.
- **Faults** latch in FAULT_STATUS and disarm at once: WDT (missed kicks),
  LINK (4 rejected frames in a row while armed), ESTOP (btnD, any time),
  ARM_LOST (SW15 dropped while armed). They raise IRQ bit 3 and hold FAULT_N
  low until cleared with FAULT_CLEAR; ESTOP can't be cleared while btnD is
  held. Clearing never re-arms: that takes a fresh key.
- **LEDs:** LED15 = armed, LED14 = fault.
- **Snapshot:** at every LOOP_TICK the FPGA latches TICK_COUNT, TIME_US, the
  inputs, the safety state and both encoders into SNAP_* (0x30–0x37; 0x38–0x3F
  are reserved for the IMU). If a frame is in progress it publishes them when
  CS rises, so one burst read of 0x30… is always one sample.

The demo firmware's 1 kHz control step reads the snapshot, writes the LEDs
and display, kicks the watchdog and clears pending IRQs, all in interrupts.
Console commands on the VCP: `a` arm, `d` disarm, `c` clear faults, `k`
stop/restart kicks (to test the watchdog).

## Motors (map v5)

- **PWM:** M1_DUTY and M2_DUTY are signed Q15 (±32767 = ±100 %). Positive
  duties PWM IN1 at 20 kHz, negative ones IN2; the other pin stays low, so the
  off-time coasts. A new duty starts at the next PWM period.
- **Safety:** both pins are gated by ARMED, so a fault or disarm turns them
  off on the same clock. The duty registers are held at 0 while disarmed, so
  arming never starts a motor. The FPGA clamps |duty| to DUTY_LIMIT, which is
  50 % after reset and can't be changed while armed.
- **Encoders:** x4 decoding (4 counts per encoder line, up when A leads B)
  after a 2-flop synchronizer and a 0.64 µs glitch filter. The snapshot holds
  each position and the counts since the previous tick. A and B changing
  together counts in ENC_ERRS, not as a step.
- **Firmware:** the control step writes WDT_KICK, M1_DUTY and M2_DUTY in one
  burst and reads the encoders with the snapshot. On the console, `m25`
  followed by Enter or a space runs M1 at 25 % (`m-40` reverses), and `s`
  stops both. The app drops the duty to 0 whenever the FPGA reports disarmed.
  A `motor:` line each second shows the duty, encoder 1 count and speed, and
  the encoder errors.

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
- Burst write, 5 + 4N bytes (N = 1–16):
  `[0x7F] [start addr] [N-1] [CRC(0..2)] [N × 4 data bytes] [CRC(data)]`.
  All-or-nothing: the words are written only if both CRCs match. A rejected
  burst is counted once in CRC_ERRS.
- Burst read, 5 + 4N bytes: MOSI `[0xFF] [start addr] [N-1] [CRC(0..2)] 00 …`;
  MISO carries the N words from byte 4 on, then a CRC over all data bytes.
  A bad header CRC or N > 16 returns all-FF data with an inverted CRC.
- Burst addresses increment from the start address and wrap at 0x7F. A burst
  counts as one frame in FRAMES_OK.
- Frames cut short by releasing CS are discarded.
- Unmapped reads return 0xDEADBEEF. Incompatible changes bump VERSION, and
  the driver's `fpga_init` refuses a mismatch.

## License

MIT, see [LICENSE](LICENSE). The vendored ST code under
`firmware/FPGA_Link/Drivers/` keeps its own licenses: STM32G4 HAL is
BSD-3-Clause, CMSIS is Apache-2.0 (see the `LICENSE.md` in each folder).
