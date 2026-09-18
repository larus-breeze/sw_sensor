#!/usr/bin/env python3
"""Build both the STM32 and ESP32 firmware images from the command line.

Thin wrapper around sw_stm32/scripts/build_firmware.py and
sw_esp32/scripts/build_firmware.py - runs both (STM32 first, then ESP32),
each with its own toolchain, and reports which ones succeeded. Both scripts
already write their final named images (packaged *.bin for both sides,
plus a named *.elf for the STM32) into the shared build/ folder at the
repository root, so afterwards both firmware images sit there together.

Usage:
    python3 build_firmware_all.py [--skip-stm32] [--skip-esp32]
                                   [--stm32-arg ARG ...] [--esp32-arg ARG ...]

Extra options for one side's build script can be forwarded with
--stm32-arg/--esp32-arg (repeatable). Use the "=" form for values that
themselves start with "--", otherwise argparse mistakes them for a new
option of this script:

    python3 build_firmware_all.py --stm32-arg=--cubeide --stm32-arg=/path/to/stm32cubeide
"""

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT_DIR = Path(__file__).resolve().parent
STM32_BUILD_SCRIPT = REPO_ROOT_DIR / "sw_stm32" / "scripts" / "build_firmware.py"
ESP32_BUILD_SCRIPT = REPO_ROOT_DIR / "sw_esp32" / "scripts" / "build_firmware.py"


def build(name, script, extra_args):
    print(f"\n{'=' * 20} Building {name} {'=' * 20}")
    result = subprocess.run([sys.executable, str(script), *extra_args])
    ok = result.returncode == 0
    print(f"{'=' * 20} {name} build {'succeeded' if ok else 'FAILED'} {'=' * 20}")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--skip-stm32", action="store_true", help="don't build the STM32 firmware")
    parser.add_argument("--skip-esp32", action="store_true", help="don't build the ESP32 firmware")
    parser.add_argument(
        "--stm32-arg",
        action="append",
        default=[],
        help="extra argument to pass to sw_stm32/scripts/build_firmware.py (repeatable)",
    )
    parser.add_argument(
        "--esp32-arg",
        action="append",
        default=[],
        help="extra argument to pass to sw_esp32/scripts/build_firmware.py (repeatable)",
    )
    args = parser.parse_args()

    results = {}
    if not args.skip_stm32:
        results["STM32"] = build("STM32", STM32_BUILD_SCRIPT, args.stm32_arg)
    if not args.skip_esp32:
        results["ESP32"] = build("ESP32", ESP32_BUILD_SCRIPT, args.esp32_arg)

    if not results:
        sys.exit("Nothing to build - both --skip-stm32 and --skip-esp32 were given.")

    print("\nSummary:")
    for name, ok in results.items():
        print(f"  {name}: {'OK' if ok else 'FAILED'}")

    if not all(results.values()):
        sys.exit(1)

    print(f"\nAll firmware images are in {REPO_ROOT_DIR / 'build'}")


if __name__ == "__main__":
    main()
