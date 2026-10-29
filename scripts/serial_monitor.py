#!/usr/bin/env python3
"""Read the Nucleo's virtual COM port and judge the link bring-up.

    python scripts/serial_monitor.py                  # 15 s, autodetect port
    python scripts/serial_monitor.py --reset          # reset the MCU first (needs STM32_Programmer_CLI)
    python scripts/serial_monitor.py --seconds 30 --port COM5
    python scripts/serial_monitor.py --list           # show serial ports

Needs pyserial (pip install pyserial). Writes everything to build/serial.log
and ends with a RESULT line:
  OK      - init OK, self-test clean, ticks counting, no CRC errors
  PARTIAL - some checks passed (details listed)
  FAILED  - no output, or init failed
Without --reset you only see the init/self-test lines if you press the
Nucleo's reset button while this runs.
"""
import argparse
import os
import re
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


def judge(lines):
    init_ok = any(l.strip() == "Init: OK" for l in lines)
    init_fail = [l.strip() for l in lines if l.startswith("Init: ") and l.strip() != "Init: OK"]
    selftests = [SELFTEST_RE.match(l) for l in lines]
    selftests = [m for m in selftests if m]
    ticks = [TICK_RE.search(l) for l in lines]
    ticks = [m for m in ticks if m]

    checks = []
    checks.append(("FPGA answered (ID + map version)", init_ok,
                   init_fail[-1] if init_fail else ("" if init_ok else "no 'Init:' line seen")))
    if selftests:
        m = selftests[-1]
        clean = m.group(1) == "OK" and all(int(m.group(i)) == 0 for i in (4, 5, 6, 7))
        checks.append(("Self-test 10,000 write/readback pairs", clean,
                       f"{m.group(1)}, mismatches {m.group(4)}, MCU CRC {m.group(5)}, "
                       f"FPGA CRC {m.group(6)}, SPI {m.group(7)}"))
    else:
        checks.append(("Self-test 10,000 write/readback pairs", False, "no self-test line seen"))

    if len(ticks) >= 2:
        a, b = ticks[0], ticks[-1]
        n = len(ticks) - 1
        mcu_rate = (int(b.group(1)) - int(a.group(1))) / n
        fpga_rate = (int(b.group(2)) - int(a.group(2))) / n
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

    print("\n--- Bring-up checks ---")
    for name, ok, detail in checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    if not lines:
        return "FAILED - no serial output (wrong port, firmware not running, or 115200 baud mismatch)"
    if all(ok for _, ok, _ in checks):
        return "OK - link up, self-test clean, ticks counting, no CRC errors"
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

    lines = []
    with serial.Serial(port, a.baud, timeout=0.2) as ser, open(log_path, "w") as log:
        ser.reset_input_buffer()
        if a.reset:
            reset_mcu()
        end = time.time() + a.seconds
        buf = b""
        while time.time() < end:
            buf += ser.read(512)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").rstrip("\r")
                lines.append(line)
                print(line)
                log.write(line + "\n")
                log.flush()

    verdict = judge(lines)
    print(f"\nRESULT: {verdict}")
    return 0 if verdict.startswith("OK") else 1


if __name__ == "__main__":
    sys.exit(main())
