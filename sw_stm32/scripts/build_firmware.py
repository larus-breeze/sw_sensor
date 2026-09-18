#!/usr/bin/env python3
"""Build the STM32 firmware from the command line via STM32CubeIDE's headless
build mode, then package it into an SD-card update image.

This wraps the same build STM32CubeIDE performs when you click "Build
Project", so the exact build configuration (Release, by default) is captured
in one command instead of depending on whichever configuration happens to be
active in a developer's local IDE - the cause of a real incident where an
outdated Debug build was packaged and flashed by mistake because the IDE
still had Debug selected.

Usage:
    python3 scripts/build_firmware.py [--config Release] [--cubeide PATH]
                                       [--workspace DIR] [--no-clean]
                                       [--no-pack] [--legacy]

Requires STM32CubeIDE (https://www.st.com/en/development-tools/stm32cubeide.html).
The launcher (stm32cubeide on Linux/macOS, stm32cubeidec.exe on Windows - the
"c" console variant only exists on Windows, where GUI executables don't
attach to a console by default) is auto-detected in common install
locations; pass --cubeide or set the STM32CUBEIDE environment variable if
it isn't found.
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SW_STM32_DIR = SCRIPT_DIR.parent
REPO_ROOT_DIR = SW_STM32_DIR.parent

DEFAULT_BUILD_CONFIG = "Release"


def run(cmd, **kwargs):
    print(f"$ {' '.join(str(c) for c in cmd)}")
    return subprocess.run(cmd, check=True, **kwargs)


def get_project_name():
    """The Eclipse/CDT project name, from .project - not necessarily the
    same as the sw_stm32 folder name, and needed to address the project in
    the headless build's "<project>/<config>" build target."""
    tree = ET.parse(SW_STM32_DIR / ".project")
    return tree.getroot().findtext("name")


def find_cubeide(explicit_path):
    if explicit_path:
        if not explicit_path.exists():
            sys.exit(f"--cubeide path does not exist: {explicit_path}")
        return explicit_path

    env_path = os.environ.get("STM32CUBEIDE")
    if env_path:
        return Path(env_path)

    # "stm32cubeidec[.exe]" (console variant) only exists on Windows; Linux
    # and macOS only ship the plain "stm32cubeide" binary, which works fine
    # for headless builds too since a terminal-launched GUI binary on those
    # platforms already writes to the calling console.
    which = (
        shutil.which("stm32cubeide")
        or shutil.which("stm32cubeidec.sh")
        or shutil.which("stm32cubeidec")
    )
    if which:
        return Path(which)

    # Common install locations across the versioned directory names
    # STM32CubeIDE installers use on each OS.
    search_globs = [
        "/opt/st/stm32cubeide_*/stm32cubeide",
        str(Path.home() / "st" / "stm32cubeide_*" / "stm32cubeide"),
        "/Applications/STM32CubeIDE.app/Contents/MacOS/stm32cubeide",
        "C:/ST/STM32CubeIDE_*/STM32CubeIDE/stm32cubeidec.exe",
        "C:/Program Files/STMicroelectronics/STM32CubeIDE_*/stm32cubeidec.exe",
    ]
    for pattern in search_globs:
        for match in sorted(glob.glob(pattern)):
            path = Path(match)
            if path.exists():
                return path

    sys.exit(
        "STM32CubeIDE launcher (stm32cubeide, or stm32cubeidec.exe on Windows) not found.\n"
        "Install it from https://www.st.com/en/development-tools/stm32cubeide.html\n"
        "or point this script at it with --cubeide /path/to/stm32cubeide\n"
        "(or set the STM32CUBEIDE environment variable)."
    )


def generate_version_header():
    # create-git-info-header.py itself uses paths relative to its own
    # working directory (Core/Inc/..., scripts/template_pack*.toml), since
    # it's normally run from a git post-commit hook with cwd=sw_stm32 - so
    # run it with that same cwd here regardless of where this script itself
    # was invoked from.
    run([sys.executable, str(SCRIPT_DIR / "create-git-info-header.py")], cwd=SW_STM32_DIR)


def run_headless_build(cubeide, workspace, project_name, config, clean):
    run(
        [
            str(cubeide),
            "--launcher.suppressErrors",
            "-nosplash",
            "-application", "org.eclipse.cdt.managedbuilder.core.headlessbuild",
            "-data", str(workspace),
            "-import", str(SW_STM32_DIR),
            "-no-indexer",
            "-cleanBuild" if clean else "-build",
            f"{project_name}/{config}",
        ]
    )


def copy_build_artifacts(config):
    """Mirror the raw build output (elf/map/list/... - whatever the config's
    build produced) from <config>/ (where STM32CubeIDE itself writes it,
    tied to the project's build configuration) into sw_stm32/build/, for
    symmetry with the ESP32 side's own raw arduino-cli output directory
    (sw_esp32/build/)."""
    src_dir = SW_STM32_DIR / config
    dest_dir = SW_STM32_DIR / "build"
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for entry in src_dir.iterdir():
        if entry.is_file():
            shutil.copy2(entry, dest_dir / entry.name)
            copied.append(entry.name)
    return dest_dir, copied


def get_version_tag():
    """Returns "MAJOR.MINOR.PATCH-BUILD" (e.g. "0.7.6-157") - the same
    git-describe parsing create-git-info-header.py uses for GIT_TAG_DEC
    and the packed image's own filename (template_pack.toml's "VERSION"
    substitution), and the same pattern the ESP32 side's own
    build_firmware.py uses to name its output. Used to name the plain
    ELF copy below as larus_sensor_stm32_v{tag}.elf.
    """
    version_string = (
        subprocess.check_output(["git", "describe", "--always", "--dirty", "--tags"], cwd=SW_STM32_DIR)
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


def run_pack(legacy=False):
    cmd = [sys.executable, str(SCRIPT_DIR / "pack.py")]
    if legacy:
        cmd.append("LEGACY")
    run(cmd)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_BUILD_CONFIG,
        help=f"STM32CubeIDE build configuration to build (default: {DEFAULT_BUILD_CONFIG})",
    )
    parser.add_argument(
        "--cubeide",
        type=Path,
        default=None,
        help="path to the stm32cubeide launcher (stm32cubeidec.exe on Windows) (default: auto-detect)",
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=None,
        help="Eclipse workspace directory to import the project into "
        "(default: a temporary directory, removed after the build)",
    )
    parser.add_argument(
        "--no-clean",
        action="store_true",
        help="incremental build (-build) instead of the default clean rebuild (-cleanBuild)",
    )
    parser.add_argument(
        "--no-pack",
        action="store_true",
        help="skip running pack.py to create the SD-card update image after building",
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="also create the LEGACY SD-card update image (pack.py LEGACY)",
    )
    args = parser.parse_args()

    cubeide = find_cubeide(args.cubeide)
    project_name = get_project_name()
    generate_version_header()

    workspace = args.workspace
    cleanup_workspace = False
    if workspace is None:
        workspace = Path(tempfile.mkdtemp(prefix="stm32cubeide_headless_ws_"))
        cleanup_workspace = True
    else:
        workspace.mkdir(parents=True, exist_ok=True)

    try:
        run_headless_build(cubeide, workspace, project_name, args.config, clean=not args.no_clean)
    finally:
        if cleanup_workspace:
            shutil.rmtree(workspace, ignore_errors=True)

    elf_path = SW_STM32_DIR / args.config / f"{project_name}.elf"
    if not elf_path.exists():
        sys.exit(f"expected output ELF not found: {elf_path}")
    print(f"\nApplication ELF: {elf_path}")

    # Named, versioned copy in the shared top-level build/ folder (alongside
    # the ESP32 side's own named outputs) - e.g. for flashing/debugging
    # directly with STM32CubeProgrammer, which reads .elf natively.
    final_build_dir = REPO_ROOT_DIR / "build"
    final_build_dir.mkdir(parents=True, exist_ok=True)
    named_elf_copy = final_build_dir / f"larus_sensor_stm32_v{get_version_tag()}.elf"
    shutil.copyfile(elf_path, named_elf_copy)
    print(f"Named ELF (e.g. for STM32CubeProgrammer): {named_elf_copy}")

    build_dir, copied = copy_build_artifacts(args.config)
    print(f"Copied build artifacts ({', '.join(copied)}) to {build_dir}")

    if not args.no_pack:
        run_pack(legacy=False)
        if args.legacy:
            run_pack(legacy=True)


if __name__ == "__main__":
    main()
