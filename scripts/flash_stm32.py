#!/usr/bin/env python3
"""Flash the Nucleo-G431KB with the link firmware.

    python scripts/flash_stm32.py                 # prebuilt/FPGA_Link.bin
    python scripts/flash_stm32.py path/to/fw.bin
    python scripts/flash_stm32.py --method drive  # force drag-and-drop copy

Method "cli" (default when found) uses STM32_Programmer_CLI over SWD, verifies
and resets the MCU. Method "drive" copies the .bin onto the ST-LINK's
NOD_G431KB USB drive, which flashes and resets the board by itself.
Prints a final RESULT line.
"""
import argparse
import glob
import os
import platform
import shutil
import string
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BIN = os.path.join(ROOT, "prebuilt", "FPGA_Link.bin")
DRIVE_LABEL = "NOD_G431KB"
FLASH_BASE = "0x08000000"


def find_cli():
    exe = "STM32_Programmer_CLI.exe" if os.name == "nt" else "STM32_Programmer_CLI"
    p = shutil.which(exe)
    if p:
        return p
    candidates = []
    if os.name == "nt":
        for base in (os.environ.get("ProgramFiles", r"C:\Program Files"),
                     os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")):
            candidates += glob.glob(os.path.join(base, "STMicroelectronics", "STM32Cube",
                                                 "STM32CubeProgrammer", "bin", exe))
        candidates += glob.glob(r"C:\ST\STM32CubeCLT*\STM32CubeProgrammer\bin\\" + exe)
        candidates += glob.glob(r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\\" + exe)
    elif platform.system() == "Darwin":
        candidates += glob.glob("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/"
                                "STM32CubeProgrammer.app/Contents/MacOs/bin/" + exe)
        candidates += glob.glob("/opt/ST/STM32CubeCLT*/STM32CubeProgrammer/bin/" + exe)
        candidates += glob.glob("/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/"
                                "*cubeprogrammer*/tools/bin/" + exe)
    else:
        home = os.path.expanduser("~")
        candidates += glob.glob(os.path.join(home, "STMicroelectronics", "STM32Cube",
                                             "STM32CubeProgrammer", "bin", exe))
        candidates += glob.glob("/opt/st/stm32cubeclt*/STM32CubeProgrammer/bin/" + exe)
        candidates += glob.glob("/opt/st/stm32cubeide*/plugins/*cubeprogrammer*/tools/bin/" + exe)
    return candidates[0] if candidates else None


def find_drive():
    if os.name == "nt":
        import ctypes
        for letter in string.ascii_uppercase:
            root = f"{letter}:\\"
            if not os.path.exists(root):
                continue
            buf = ctypes.create_unicode_buffer(261)
            ok = ctypes.windll.kernel32.GetVolumeInformationW(
                ctypes.c_wchar_p(root), buf, 261, None, None, None, None, 0)
            if ok and buf.value.upper() == DRIVE_LABEL:
                return root
        return None
    patterns = [f"/Volumes/{DRIVE_LABEL}", f"/media/*/{DRIVE_LABEL}",
                f"/run/media/*/{DRIVE_LABEL}", f"/media/{DRIVE_LABEL}"]
    for pat in patterns:
        hits = glob.glob(pat)
        if hits:
            return hits[0]
    return None


def flash_cli(cli, image):
    cmd = [cli, "-c", "port=SWD", "mode=UR", "reset=HWrst"]
    if image.lower().endswith(".bin"):
        cmd += ["-w", image, FLASH_BASE]
    else:
        cmd += ["-w", image]
    cmd += ["-v", "-rst"]
    print("Running:", " ".join(f'"{c}"' if " " in c else c for c in cmd))
    r = subprocess.run(cmd)
    return r.returncode == 0


def flash_drive(drive, image):
    if not image.lower().endswith(".bin"):
        print("Drag-and-drop flashing needs a .bin file")
        return False
    dest = os.path.join(drive, os.path.basename(image))
    print(f"Copying {image} -> {dest}")
    shutil.copyfile(image, dest)
    # The ST-LINK flashes, then re-enumerates the drive (~2-5 s)
    time.sleep(6)
    fail = os.path.join(drive, "FAIL.TXT")
    if os.path.exists(fail):
        with open(fail, errors="replace") as f:
            print("ST-LINK reported:", f.read().strip())
        return False
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", nargs="?", default=DEFAULT_BIN)
    ap.add_argument("--method", choices=["auto", "cli", "drive"], default="auto")
    a = ap.parse_args()

    image = os.path.abspath(a.image)
    if not os.path.exists(image):
        print(f"RESULT: FAILED - image not found: {image}")
        return 1

    cli = find_cli() if a.method in ("auto", "cli") else None
    if cli:
        ok = flash_cli(cli, image)
        print("RESULT: OK - flashed with STM32_Programmer_CLI" if ok else
              "RESULT: FAILED - STM32_Programmer_CLI returned an error (see output above)")
        return 0 if ok else 1
    if a.method == "cli":
        print("RESULT: FAILED - STM32_Programmer_CLI not found (install STM32CubeProgrammer or use --method drive)")
        return 1

    drive = find_drive()
    if not drive:
        print(f"RESULT: FAILED - neither STM32_Programmer_CLI nor the {DRIVE_LABEL} drive was found. "
              "Is the Nucleo plugged in?")
        return 1
    ok = flash_drive(drive, image)
    print(f"RESULT: OK - copied to {drive} (board resets itself)" if ok else
          "RESULT: FAILED - drag-and-drop flash did not complete")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
