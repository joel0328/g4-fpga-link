#!/usr/bin/env python3
"""Read the Nucleo's virtual COM port and judge the link bring-up.

    python scripts/serial_monitor.py                  # 15 s, autodetect port
    python scripts/serial_monitor.py --reset          # reset the MCU first (needs STM32_Programmer_CLI)
    python scripts/serial_monitor.py --seconds 30 --port COM5
    python scripts/serial_monitor.py --list           # show serial ports
    python scripts/serial_monitor.py --reset --arm    # also arm (SW15 must be up); expects armed
    python scripts/serial_monitor.py --cmd 20:k --cmd 25:c   # console bytes at t = 20 s, 25 s

Needs pyserial (pip install pyserial). Writes everything to build/serial.log
and ends with a RESULT line:
  OK      - init OK, both self-tests clean, ticks counting, no CRC errors,
            no safety fault (and armed, with --arm)
  PARTIAL - some checks passed (details listed)
  FAILED  - no output, or init failed
Without --reset you only see the init/self-test lines if you press the
Nucleo's reset button while this runs.
"""
import argparse
import os
import re
import statistics
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("RESULT: FAILED - pyserial is not installed (pip install pyserial)")
    sys.exit(2)

ST_VID = 0x0483

TICK_RE = re.compile(r"ticks MCU (\d+) / FPGA (\d+), period (\d+)\.\.(\d+) ns, frames (\d+), "
                     r"FPGA CRC errs (\d+), MCU CRC errs (\d+), IRQs (\d+)")
SELFTEST_RE = re.compile(r"^\s*(\S.*?): (\d+) iterations in (\d+) ms, mismatches (\d+), "
                         r"MCU CRC errors (\d+), FPGA CRC errors (\d+), SPI errors (\d+)")
STEP_RE = re.compile(r"step: done (\d+), skipped (\d+), errors (\d+), latency (\d+)\.\.(\d+) ns "
                     r"\(avg (\d+)\), callback CPU avg (\d+) ns")
SAFETY_RE = re.compile(r"safety: (armed|disarmed), SW15 (up|down), kicks (on|off), faults 0x([0-9A-F]+)")
BOOT_DISARMED = "safety: disarmed; SW15 up"   # the firmware is ready for 'a'


def find_port():
    ports = list(serial.tools.list_ports.comports())
    st = [p for p in ports if p.vid == ST_VID]
    if st:
        return st[0].device
    for p in ports:
        if "STLink" in (p.description or "") or "STMicro" in (p.manufacturer or ""):
            return p.device
    return None


def reset_mcu():
    from flash_stm32 import find_cli
    cli = find_cli()
    if not cli:
        print("(--reset: STM32_Programmer_CLI not found; press the Nucleo reset button now)")
        return False
    r = subprocess.run([cli, "-c", "port=SWD", "mode=HOTPLUG", "-hardRst"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode != 0:
        print("(--reset: STM32_Programmer_CLI reset failed; press the Nucleo reset button now)")
        return False
    return True


SELFTESTS = (("single", "Self-test:", "Self-test 10,000 write/readback pairs"),
             ("burst", "Burst self-test:", "Burst self-test 2,000 x 16-word bursts"))


def parse_selftests(lines):
    """Last result line under each self-test header, keyed by kind."""
    found, kind = {}, None
    for l in lines:
        header = next((k for k, prefix, _ in SELFTESTS if l.startswith(prefix)), None)
        if header:
            kind = header
            continue
        m = SELFTEST_RE.match(l)
        if m and kind:
            found[kind] = m
    return found


def judge(lines, expect_armed=False):
    init_ok = any(l.strip() == "Init: OK" for l in lines)
    init_fail = [l.strip() for l in lines if l.startswith("Init: ") and l.strip() != "Init: OK"]
    selftests = parse_selftests(lines)
    ticks = [TICK_RE.search(l) for l in lines]
    ticks = [m for m in ticks if m]
    steps = [STEP_RE.search(l) for l in lines]
    steps = [m for m in steps if m]
    safety = [SAFETY_RE.search(l) for l in lines]
    safety = [m for m in safety if m]
    faults = [l.strip() for l in lines if l.startswith("SAFETY FAULT")]

    checks = []
    checks.append(("FPGA answered (ID + map version)", init_ok,
                   init_fail[-1] if init_fail else ("" if init_ok else "no 'Init:' line seen")))
    for kind, _, name in SELFTESTS:
        m = selftests.get(kind)
        if m:
            clean = m.group(1) == "OK" and all(int(m.group(i)) == 0 for i in (4, 5, 6, 7))
            checks.append((name, clean,
                           f"{m.group(1)}, mismatches {m.group(4)}, MCU CRC {m.group(5)}, "
                           f"FPGA CRC {m.group(6)}, SPI {m.group(7)}"))
        else:
            checks.append((name, False, "no result line seen"))

    if len(ticks) >= 2:
        b = ticks[-1]
        # Per-line rates (lines are ~1 s apart). Skip intervals where a count
        # went backwards: btnC clears both counters on purpose.
        deltas = [(int(y.group(1)) - int(x.group(1)), int(y.group(2)) - int(x.group(2)))
                  for x, y in zip(ticks, ticks[1:])]
        deltas = [d for d in deltas if d[0] >= 0 and d[1] >= 0] or [(0, 0)]
        mcu_rate = statistics.median(d[0] for d in deltas)
        fpga_rate = statistics.median(d[1] for d in deltas)
        lo, hi = int(b.group(3)), int(b.group(4))
        ok_rate = 900 <= mcu_rate <= 1100 and abs(mcu_rate - fpga_rate) <= 5
        checks.append(("LOOP_TICK ~1 kHz, MCU and FPGA agree", ok_rate,
                       f"MCU {mcu_rate:.0f}/s, FPGA {fpga_rate:.0f}/s, period {lo}..{hi} ns"))
        crc = int(b.group(6)) + int(b.group(7))
        checks.append(("No CRC errors while running", crc == 0,
                       f"FPGA CRC errs {b.group(6)}, MCU CRC errs {b.group(7)}, frames {b.group(5)}"))
    else:
        checks.append(("LOOP_TICK ~1 kHz, MCU and FPGA agree", False,
                       f"only {len(ticks)} status line(s) seen; run longer"))

    if len(steps) >= 2:
        b = steps[-1]
        # Median per-line rate, as for ticks: skip resets (btnC, or a stale
        # line from before --reset took effect)
        deltas = [int(y.group(1)) - int(x.group(1)) for x, y in zip(steps, steps[1:])]
        rate = statistics.median([d for d in deltas if d >= 0] or [0])
        clean = int(b.group(2)) == 0 and int(b.group(3)) == 0 and 900 <= rate <= 1100
        checks.append(("Control step on every tick (SPI DMA)", clean,
                       f"{rate:.0f}/s, skipped {b.group(2)}, errors {b.group(3)}, "
                       f"latency {b.group(4)}..{b.group(5)} ns (avg {b.group(6)}), "
                       f"callback CPU avg {b.group(7)} ns"))
    else:
        checks.append(("Control step on every tick (SPI DMA)", False,
                       f"only {len(steps)} step line(s) seen"))

    name = "Safety armed, no fault" if expect_armed else "Safety: no fault latched"
    if safety:
        m = safety[-1]
        clean = m.group(4) == "0" and not faults and (m.group(1) == "armed" or not expect_armed)
        checks.append((name, clean,
                       f"{m.group(1)}, SW15 {m.group(2)}, kicks {m.group(3)}, faults 0x{m.group(4)}"
                       + (f"; {len(faults)} SAFETY FAULT line(s): {faults[-1]}" if faults else "")))
    else:
        checks.append((name, False, "no safety line seen"))

    print("\n--- Bring-up checks ---")
    for name, ok, detail in checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    if not lines:
        return "FAILED - no serial output (wrong port, firmware not running, or 115200 baud mismatch)"
    if all(ok for _, ok, _ in checks):
        return "OK - link up, self-tests clean, ticks counting, no CRC errors"
    if not init_ok and not ticks:
        return "FAILED - FPGA did not answer; see the checks above and the troubleshooting table"
    return "PARTIAL - " + "; ".join(n for n, ok, _ in checks if not ok) + " failed"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--reset", action="store_true", help="reset the MCU after opening the port")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--arm", action="store_true",
                    help="send 'a' once the firmware is up (SW15 must be up) and expect it armed")
    ap.add_argument("--cmd", action="append", default=[], metavar="SECONDS:CHAR",
                    help="send a console byte at a time into the run, e.g. 20:k (repeatable)")
    a = ap.parse_args()

    if a.list:
        for p in serial.tools.list_ports.comports():
            vid = f"{p.vid:04X}" if p.vid is not None else "----"
            print(f"{p.device:15} VID {vid}  {p.description}")
        return 0

    port = a.port or find_port()
    if not port:
        print("RESULT: FAILED - no ST-LINK virtual COM port found (use --list / --port)")
        return 1

    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    log_path = os.path.join(ROOT, "build", "serial.log")
    print(f"Listening on {port} at {a.baud} baud for {a.seconds:.0f} s (log: {log_path})")

    cmds = sorted((float(s.split(":", 1)[0]), s.split(":", 1)[1].encode()) for s in a.cmd)
    lines = []
    with serial.Serial(port, a.baud, timeout=0.2) as ser, open(log_path, "w") as log:
        ser.reset_input_buffer()
        if a.reset:
            reset_mcu()
        start = time.time()
        end = start + a.seconds
        arm_pending = a.arm
        if arm_pending and not a.reset:      # firmware already running
            ser.write(b"a")
            arm_pending = False
        buf = b""
        while time.time() < end:
            while cmds and time.time() - start >= cmds[0][0]:
                ser.write(cmds.pop(0)[1])
            buf += ser.read(512)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").rstrip("\r")
                lines.append(line)
                print(line)
                log.write(line + "\n")
                log.flush()
                if arm_pending and line.startswith(BOOT_DISARMED):
                    ser.write(b"a")
                    arm_pending = False

    verdict = judge(lines, expect_armed=a.arm)
    print(f"\nRESULT: {verdict}")
    return 0 if verdict.startswith("OK") else 1


if __name__ == "__main__":
    sys.exit(main())
