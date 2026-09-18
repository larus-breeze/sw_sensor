#!/usr/bin/env python3
"""Build a flashable/OTA-able binary of the ESP32 sketch using arduino-cli.

This wraps the same compile step the Arduino IDE performs, so the exact
board configuration (board, flash size, partition scheme) is captured in one
place instead of living only in a developer's local IDE settings.

Usage:
    python3 scripts/build_firmware.py [--output-dir DIR] [--flash-size SIZE]
                                       [--partition-scheme SCHEME] [--fqbn FQBN]

Requires arduino-cli (https://arduino.github.io/arduino-cli/latest/installation/)
with the esp32:esp32 board core installed:
    arduino-cli config add board_manager.additional_urls \
        https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    arduino-cli core update-index
    arduino-cli core install esp32:esp32

Also requires esptool (pip install esptool, or the venv from "Python
environment setup" in the ESP32 README) to merge the individual pieces
(bootloader/partitions/boot_app0/app) down to two flat images for
esptool-js - see "Initial flash without a local toolchain" there. Split
in two, not merged into one, so the untouched gap between them (the NVS
partition - WiFi credentials, AP identity) is never overwritten.
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SW_ESP32_DIR = SCRIPT_DIR.parent
REPO_ROOT_DIR = SW_ESP32_DIR.parent
SKETCH_DIR = SW_ESP32_DIR / "wlan_link"
SKETCH_NAME = SKETCH_DIR.name

# Matches the "ESP32 Dev Module" board with an 8 MByte flash chip, as
# documented in sw_esp32/README.md for manual Arduino IDE flashing.
DEFAULT_CORE_FQBN = "esp32:esp32:esp32"
DEFAULT_FLASH_SIZE = "8M"
# "default" (Default 4MB with spiffs, 1.25 MB app partition) is too small -
# the compiled sketch was ~1.58 MB with Bluetooth Classic included, which
# overflowed it ("text section exceeds available space in board"). OTA
# updatability matters here (unlike a one-off "no OTA" scheme), so instead
# of dropping OTA support, Bluetooth Classic (BluetoothSerial) was removed
# from uart_bridge.cpp - WiFi-only for now, see its doc comment - to fit
# "min_spiffs" (Minimal SPIFFS: 1.9 MB app partition per OTA slot,
# 190 KB SPIFFS), which keeps the OTA_0/OTA_1 partition pair.
DEFAULT_PARTITION_SCHEME = "min_spiffs"

OUTPUT_NAME_TEMPLATE = "larus_sensor_esp32_v{}.bin"
MERGED_BOOT_NAME_TEMPLATE = "larus_sensor_esp32_v{}.flash1_boot.bin"
MERGED_APP_NAME_TEMPLATE = "larus_sensor_esp32_v{}.flash2_app.bin"

# Standard Arduino-ESP32 offsets (same regardless of partition scheme,
# hence hardcoded rather than parsed out of partitions.bin): the NVS
# partition - WiFi credentials, AP identity (ap_identity.cpp) - lives at
# 0x9000-0xe000, between the partition table and boot_app0.bin/otadata.
# merge_bin fills every byte between its first and last offset, including
# gaps, with 0xff - merging straight across this one would silently erase
# NVS on every reflash of an already-configured board, not just a blank
# one. Split into two merges instead, stopping/resuming right at its
# boundary, so NVS is never part of either merged image.
NVS_PARTITION_START = 0x9000
NVS_PARTITION_END = 0xe000

# Standard Arduino-ESP32 defaults for an "ESP32 Dev Module" board - same
# caveat as the offset table this feeds into (README's "Initial flash
# without a local toolchain"): confirm via `arduino-cli compile --verbose`
# if a different board variant is ever used here.
ESPTOOL_FLASH_MODE = "dio"
ESPTOOL_FLASH_FREQ = "40m"


def run(cmd, **kwargs):
    print(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, check=True, **kwargs)


def check_arduino_cli_installed():
    if shutil.which("arduino-cli") is None:
        sys.exit(
            "arduino-cli not found on PATH.\n"
            "Install it from https://arduino.github.io/arduino-cli/latest/installation/"
        )


def generate_version_header():
    run([sys.executable, str(SCRIPT_DIR / "create-git-info-header.py")])


def get_version_tag():
    """Returns "MAJOR.MINOR.PATCH-BUILD" (e.g. "0.7.6-157"), mirroring the
    STM32 side's scripts/create-git-info-header.py's own tag/build-number
    parsing (same regex, same "no tag reachable" fallback) - used to name
    the output binary as larus_sensor_esp32_v{tag}.bin, matching the STM32
    side's own larus_sensor_stm32_v{tag}.bin pattern (scripts/pack.py, run
    from the sw_stm32 side) closely enough to tell at a glance which pair
    of binaries belongs together.
    """
    version_string = (
        subprocess.check_output(["git", "describe", "--always", "--dirty", "--tags"], cwd=SCRIPT_DIR)
        .decode("utf-8")
        .strip()
    )
    match = re.match(r"(\d+)\.(\d+)\.(\d+)-(\d+)-", version_string)
    if match:
        return "{}.{}.{}-{}".format(*match.groups())
    match = re.match(r"(\d+)\.(\d+)\.(\d+)", version_string)
    if match:
        return "{}.{}.{}-0".format(*match.groups())
    return "unknown"


def check_esp32_core_installed():
    result = subprocess.run(
        ["arduino-cli", "core", "list", "--format", "json"],
        check=True,
        capture_output=True,
        text=True,
    )
    data = json.loads(result.stdout or "[]")
    # `arduino-cli core list --format json`'s output shape has changed
    # across versions: older releases returned a bare array of core
    # objects, newer ones wrap it as {"platforms": [...]} - and the core's
    # own id key has been seen as both "id" and "ID". Handle both.
    cores = data.get("platforms", []) if isinstance(data, dict) else data
    if any(core.get("id", core.get("ID")) == "esp32:esp32" for core in cores):
        return

    sys.exit(
        "esp32:esp32 board core is not installed for arduino-cli. Install it with:\n\n"
        "  arduino-cli config add board_manager.additional_urls "
        "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json\n"
        "  arduino-cli core update-index\n"
        "  arduino-cli core install esp32:esp32\n"
    )


def check_esptool_installed():
    try:
        subprocess.run(
            [sys.executable, "-m", "esptool", "version"],
            check=True,
            capture_output=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        sys.exit(
            "esptool not installed in this Python environment.\n"
            "Install it with the venv from \"Python environment setup\" in "
            "sw_esp32/README.md (pip install -r scripts/requirements.txt), "
            "or standalone with: pip install esptool"
        )


def find_boot_app0_bin():
    """Locates the generic OTA-slot-selector stub the bootloader needs,
    bundled with the installed esp32:esp32 core rather than produced by
    this build (see the doc comment where it's used, below)."""
    data_dir = Path.home() / ".arduino15"  # arduino-cli's documented default
    try:
        config = json.loads(
            subprocess.run(
                ["arduino-cli", "config", "dump", "--format", "json"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
        data_dir = Path(config["directories"]["data"])
    except (subprocess.CalledProcessError, FileNotFoundError, KeyError, json.JSONDecodeError):
        pass  # fall back to the default above - still worth trying

    matches = sorted(data_dir.glob("packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin"))
    if not matches:
        sys.exit(
            f"boot_app0.bin not found under {data_dir} (esp32:esp32 core "
            "installed?). See \"Initial flash without a local toolchain\" "
            "in sw_esp32/README.md for where it normally lives."
        )
    return matches[-1]  # highest installed core version, if more than one


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=SW_ESP32_DIR / "build",
        help="directory to write the raw compiled binaries to (default: sw_esp32/build)",
    )
    parser.add_argument(
        "--flash-size",
        default=DEFAULT_FLASH_SIZE,
        help=f"board FlashSize option (default: {DEFAULT_FLASH_SIZE})",
    )
    parser.add_argument(
        "--partition-scheme",
        default=DEFAULT_PARTITION_SCHEME,
        help=f"board PartitionScheme option (default: {DEFAULT_PARTITION_SCHEME})",
    )
    parser.add_argument(
        "--fqbn",
        default=DEFAULT_CORE_FQBN,
        help=f"board core FQBN, board options are appended (default: {DEFAULT_CORE_FQBN})",
    )
    args = parser.parse_args()

    check_arduino_cli_installed()
    check_esp32_core_installed()
    check_esptool_installed()
    boot_app0_binary = find_boot_app0_bin()  # fail fast, before the compile below
    generate_version_header()

    fqbn = f"{args.fqbn}:FlashSize={args.flash_size},PartitionScheme={args.partition_scheme}"

    args.output_dir.mkdir(parents=True, exist_ok=True)

    run(
        [
            "arduino-cli",
            "compile",
            "--fqbn",
            fqbn,
            "--export-binaries",
            "--output-dir",
            str(args.output_dir),
            str(SKETCH_DIR),
        ]
    )

    # arduino-cli --export-binaries writes <sketch>.ino.bin (application
    # image, for OTA / re-flashing over an existing bootloader+partitions)
    # plus .bootloader.bin, .partitions.bin and .elf (needed only for a
    # from-scratch flash of a blank chip, e.g. via esptool.py write_flash).
    app_binary = args.output_dir / f"{SKETCH_NAME}.ino.bin"
    bootloader_binary = args.output_dir / f"{SKETCH_NAME}.ino.bootloader.bin"
    partitions_binary = args.output_dir / f"{SKETCH_NAME}.ino.partitions.bin"
    for expected in (app_binary, bootloader_binary, partitions_binary):
        if not expected.exists():
            sys.exit(f"expected output binary not found: {expected}")

    # partitions.bin must end at or before NVS_PARTITION_START, or the
    # "boot" merge below would silently reach into NVS itself - see the
    # module-level comment. True for every stock Arduino-ESP32 partition
    # table (partitions.bin is always <= one 4KB sector), checked here
    # rather than just assumed.
    partitions_end = 0x8000 + partitions_binary.stat().st_size
    if partitions_end > NVS_PARTITION_START:
        sys.exit(
            f"partitions.bin ends at 0x{partitions_end:x}, past NVS's own "
            f"start (0x{NVS_PARTITION_START:x}) - merging would overwrite "
            "part of NVS (WiFi credentials, AP identity). Not merging."
        )

    flash_size = args.flash_size if args.flash_size.upper().endswith("MB") else f"{args.flash_size.upper()}B"

    def merge_bin(output_path, *offset_file_pairs):
        # Without --target-offset, merge_bin always builds the output as if
        # it will be flashed at 0x0 - even here, where the lowest given
        # offset is 0x1000 or 0xe000, it still pads the file with 0xff from
        # 0x0 up to that offset rather than starting there. Flashing such a
        # file at anything other than 0x0 (as the offset table below does)
        # shifts every piece in it to the wrong address - nothing ends up
        # at the real 0x10000 app slot, which is exactly what produced the
        # "invalid header: 0xffffffff" boot loop this was debugged from.
        # --target-offset anchors the output at the given offset instead,
        # both trimming that dead padding (much faster to flash) and
        # making its own offsets add up correctly.
        run(
            [
                sys.executable, "-m", "esptool",
                "--chip", "esp32",
                "merge-bin",
                "-o", str(output_path),
                "--target-offset", offset_file_pairs[0][0],
                "--flash-mode", ESPTOOL_FLASH_MODE,
                "--flash-freq", ESPTOOL_FLASH_FREQ,
                "--flash-size", flash_size,
                *(str(part) for pair in offset_file_pairs for part in pair),
            ]
        )

    # Two merged images instead of one, split at the NVS boundary (see the
    # module-level comment) - flashed at their own offsets in esptool-js,
    # same as the original four separate files, just down to two.
    boot_binary = args.output_dir / f"{SKETCH_NAME}.ino.flash1_boot.bin"
    merge_bin(boot_binary, ("0x1000", bootloader_binary), ("0x8000", partitions_binary))
    app_flash_binary = args.output_dir / f"{SKETCH_NAME}.ino.flash2_app.bin"
    merge_bin(app_flash_binary, (hex(NVS_PARTITION_END), boot_app0_binary), ("0x10000", app_binary))

    # The raw arduino-cli output (bootloader/partitions/elf/map, only needed
    # for flashing a completely blank chip by hand) stays in --output-dir;
    # only the final named images - the deliverables this produces - also
    # go into the shared top-level build/ folder, alongside the STM32
    # side's own packaged image (sw_stm32/scripts/pack.py).
    final_build_dir = REPO_ROOT_DIR / "build"
    final_build_dir.mkdir(parents=True, exist_ok=True)
    named_copy = final_build_dir / OUTPUT_NAME_TEMPLATE.format(get_version_tag())
    shutil.copyfile(app_binary, named_copy)
    named_boot_copy = final_build_dir / MERGED_BOOT_NAME_TEMPLATE.format(get_version_tag())
    shutil.copyfile(boot_binary, named_boot_copy)
    named_app_copy = final_build_dir / MERGED_APP_NAME_TEMPLATE.format(get_version_tag())
    shutil.copyfile(app_flash_binary, named_app_copy)

    print(f"\nApplication binary (for OTA updates): {named_copy}")
    print("Flash images for a from-scratch flash (esptool-js or esptool.py write_flash),")
    print("NVS/WiFi credentials untouched - see sw_esp32/README.md:")
    print(f"  0x1000  {named_boot_copy}")
    print(f"  0x{NVS_PARTITION_END:x}  {named_app_copy}")
    print(f"All build outputs (incl. bootloader/partitions for a full flash): {args.output_dir}")


if __name__ == "__main__":
    main()
