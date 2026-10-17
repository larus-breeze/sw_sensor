#!/usr/bin/env python3
"""Build the STM32 firmware without STM32CubeIDE, for CI checks.

Reads the build configuration (Release by default) from sw_stm32/.cproject -
sources, excluded paths, include paths, defines, optimization and warning
options including the per-folder overrides - and builds it with the
arm-none-eabi toolchain found on PATH: compile, link, .bin, .map, memory
usage per region, compiler warnings.

This is a check, not a release build: the Arm GNU Toolchain is not identical
to ST's "GNU Tools for STM32" that STM32CubeIDE uses, so the binary is not
bit-identical to a CubeIDE build. Release images come from build_firmware.py.

The flags STM32CubeIDE adds on its own (not stored in .cproject) are listed in
CUBEIDE_* below; they match the DW_AT_producer strings of the 0.7.6 release.

Usage (from sw_stm32/ or anywhere):
    python3 scripts/ci_build.py [--config Release] [--build-dir DIR]
                                [--json FILE] [--warn-usage PERCENT]
                                [--fail-usage REGION=PERCENT ...]
"""

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SW_STM32_DIR = SCRIPT_DIR.parent
PREFIX = "arm-none-eabi-"

MCU_CPU = {"STM32F4": "cortex-m4"}
CUBEIDE_COMMON = ["--specs=nano.specs", "-ffunction-sections", "-fdata-sections", "-Wall", "-fstack-usage"]
CUBEIDE_ST_ONLY = ["-fcyclomatic-complexity"]  # GNU Tools for STM32 extension, dropped if unsupported
CUBEIDE_C = ["-std=gnu11"]
CUBEIDE_CXX = ["-std=gnu++14", "-fno-exceptions", "-fno-rtti", "-fno-use-cxa-atexit"]
CUBEIDE_ASM = ["-x", "assembler-with-cpp"]
CUBEIDE_LINK = ["-Wl,--gc-sections", "-static", "--specs=nano.specs",
                "-Wl,--start-group", "-lc", "-lm", "-lstdc++", "-lsupc++", "-Wl,--end-group"]

WARNING_OPTIONS = {
    "warnings.extra": "-Wextra",
    "warnings.pedanticerrors": "-pedantic-errors",
    "warnings.missing_include_dirs": "-Wmissing-include-dirs",
    "warnings.w_switch_default": "-Wswitch-default",
    "warnings.w_switch_enum": "-Wswitch-enum",
    "warnings.conversion": "-Wconversion",
}
KNOWN_OPTIONS = {"optimization.level", "debuglevel", "includepaths", "definedsymbols", "otherflags",
                 "script", "libraries", "directories", "libmath", "systemcalls"} | set(WARNING_OPTIONS)


def resolve(path):
    """Project paths in .cproject: ${workspace_loc:/${ProjName}/X} or ../X (relative to the build dir)."""
    path = path.strip('"')
    m = re.match(r"\$\{workspace_loc:/\$\{ProjName\}/(.*)\}$", path)
    if m:
        return SW_STM32_DIR / m.group(1)
    if path.startswith("../"):
        return SW_STM32_DIR / path[3:]
    return SW_STM32_DIR / path


def tool_kind(super_class):
    for kind in ("cpp.compiler", "c.compiler", "assembler", "cpp.linker", "c.linker"):
        if ".tool." + kind in super_class:
            return kind
    return None


def option_key(super_class):
    return super_class.split(".option.")[-1]


class Config:
    def __init__(self, name):
        root = ET.parse(SW_STM32_DIR / ".cproject").getroot()
        self.cfg = next((c for c in root.iter("configuration") if c.get("name") == name), None)
        if self.cfg is None:
            sys.exit(f"configuration {name!r} not found in .cproject")
        self.prebuild = self.cfg.get("prebuildStep")
        self.postbuild = self.cfg.get("postbuildStep")
        # resourcePath -> tool kind -> option key -> (value, [list values])
        self.folders = {}
        self.toolchain = {}
        for fi in self.cfg.iter("folderInfo"):
            rp = fi.get("resourcePath") or ""
            tools = self.folders.setdefault(rp, {})
            tc = fi.find("toolChain")
            if tc is not None and rp == "":
                for o in tc.findall("option"):
                    self.toolchain[option_key(o.get("superClass"))] = o.get("value")
            for tool in fi.iter("tool"):
                kind = tool_kind(tool.get("superClass", ""))
                if not kind:
                    continue
                opts = tools.setdefault(kind, {})
                for o in tool.findall("option"):
                    key = option_key(o.get("superClass"))
                    opts[key] = (o.get("value"), [l.get("value") for l in o.iter("listOptionValue")])
        self.unknown = sorted({k for t in self.folders.values() for o in t.values() for k in o} - KNOWN_OPTIONS)

    def options(self, rel_path, kind):
        """Options for a source file: project defaults, overridden by more specific folders."""
        merged = {}
        for rp in sorted(self.folders, key=len):
            if rp == "" or rel_path == rp or rel_path.startswith(rp + "/"):
                merged.update(self.folders[rp].get(kind, {}))
        return merged

    def sources(self):
        files = []
        for entry in self.cfg.iter("entry"):
            if entry.get("kind") != "sourcePath":
                continue
            base = entry.get("name")
            excluded = [f"{base}/{x}" for x in (entry.get("excluding") or "").split("|") if x]
            for dirpath, _, names in os.walk(SW_STM32_DIR / base):
                for n in names:
                    if not n.endswith((".c", ".cpp", ".s", ".S")):
                        continue
                    rel = (Path(dirpath) / n).relative_to(SW_STM32_DIR).as_posix()
                    if any(rel == x or rel.startswith(x + "/") for x in excluded):
                        continue
                    files.append(rel)
        return sorted(set(files))

    def target_flags(self):
        mcu = self.toolchain.get("target_mcu", "")
        cpu = next((c for prefix, c in MCU_CPU.items() if mcu.startswith(prefix)), None)
        if not cpu:
            sys.exit(f"unknown target MCU {mcu!r}, extend MCU_CPU")
        fpu = self.toolchain.get("fpu", "").split(".value.")[-1]
        abi = self.toolchain.get("floatabi", "").split(".value.")[-1]
        return [f"-mcpu={cpu}", f"-mfpu={fpu}", f"-mfloat-abi={abi}", "-mthumb"]


def level(value, prefix):
    """...optimization.level.value.o3 -> -O3, ...debuglevel.value.g0 -> -g0"""
    v = (value or "").split(".value.")[-1]
    return "-" + (v[0].upper() + v[1:] if prefix == "O" else v) if v else None


def compile_flags(cfg, rel, kind, target, extra, out):
    o = cfg.options(rel, kind)
    flags = list(target)
    for key in ("optimization.level", "debuglevel"):
        if key in o:
            flags.append(level(o[key][0], "O" if key.startswith("opt") else "g"))
    flags += ["-I" + os.path.relpath(resolve(p), out) for p in o.get("includepaths", (None, []))[1]]
    flags += ["-D" + re.sub(r"\s*=\s*", "=", d.strip()) for d in o.get("definedsymbols", (None, []))[1]]
    for key, flag in WARNING_OPTIONS.items():
        if o.get(key, ("false",))[0] == "true":
            flags.append(flag)
    flags += shlex.split(o.get("otherflags", ("", []))[0] or "")
    flags += [x for x in o.get("otherflags", (None, []))[1] if x]
    if kind == "assembler":
        return flags + CUBEIDE_ASM
    return flags + CUBEIDE_COMMON + extra + (CUBEIDE_CXX if kind == "cpp.compiler" else CUBEIDE_C)


def supported(flag):
    r = subprocess.run([PREFIX + "gcc", flag, "-x", "c", "-c", "-o", os.devnull, "-"],
                       input="", capture_output=True, text=True)
    return r.returncode == 0


DIAG = re.compile(r"^(?P<file>[^:\s][^:]*):(?P<line>\d+):(?:\d+:)? (?P<kind>warning|error): (?P<msg>.*?)(?: \[(?P<flag>-W[^\]]+|-f[^\]]+)\])?$", re.M)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default="Release")
    ap.add_argument("--build-dir", type=Path, help="default: sw_stm32/<config>-ci")
    ap.add_argument("--json", type=Path, help="write a summary (memory usage, warnings, errors)")
    ap.add_argument("--warn-usage", type=float, default=90.0, help="report regions above this usage in percent")
    ap.add_argument("--fail-usage", action="append", default=[], metavar="REGION=PERCENT",
                    help="fail if a memory region is used above PERCENT, e.g. FLASH=95")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    cfg = Config(args.config)
    out = args.build_dir or SW_STM32_DIR / f"{args.config}-ci"
    out.mkdir(parents=True, exist_ok=True)
    project = "sw_sensor"
    version = subprocess.run([PREFIX + "gcc", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    print(version)
    if cfg.unknown:
        print("note: .cproject options not mapped by this script:", ", ".join(cfg.unknown))

    if cfg.prebuild:  # CubeIDE runs it in the build directory, e.g. "cd .. && bash scripts/..."
        subprocess.run(cfg.prebuild, shell=True, cwd=out, check=True)

    target = cfg.target_flags()
    extra = [f for f in CUBEIDE_ST_ONLY if supported(f)]
    sources = cfg.sources()

    def build(rel):
        kind = "cpp.compiler" if rel.endswith(".cpp") else "assembler" if rel.endswith((".s", ".S")) else "c.compiler"
        obj = out / (rel + ".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        cc = PREFIX + ("g++" if kind == "cpp.compiler" else "gcc")
        # relative paths like CubeIDE ("../Core/Src/x.c"): __FILE__ strings end up in the binary
        src = os.path.relpath(SW_STM32_DIR / rel, out)
        cmd = [cc] + compile_flags(cfg, rel, kind, target, extra, out) + ["-c", src, "-o", str(obj)]
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=out)
        return rel, obj, r.returncode, r.stderr

    with ThreadPoolExecutor(args.jobs) as ex:
        results = list(ex.map(build, sources))

    log = out / "compile.log"
    failed = [rel for rel, _, rc, _ in results if rc]
    diags = []
    with open(log, "w") as f:
        for rel, _, rc, err in results:
            if err:
                f.write(f"=== {rel}\n{err}\n")
            for m in DIAG.finditer(err):
                diags.append((m.group("kind"), m.group("flag") or "(no flag)",
                              os.path.relpath(os.path.normpath(os.path.join(out, m.group("file"))), SW_STM32_DIR),
                              int(m.group("line")), m.group("msg")))
    unique = sorted(set(diags))
    by_flag = Counter((k, fl) for k, fl, *_ in unique)
    print(f"{len(sources)} sources, {len(failed)} failed, "
          f"{sum(1 for d in unique if d[0] == 'warning')} unique warnings, full log: {log}")
    for (kind, flag), n in sorted(by_flag.items(), key=lambda kv: (kv[0][0] != "error", -kv[1])):
        print(f"  {kind:7s} {flag:32s} {n}")
    errors = {rel: err for rel, _, rc, err in results if rc}
    for rel in failed:
        print(f"FAILED {rel}")
        print("\n".join(l for l in errors[rel].splitlines() if ": error:" in l))

    summary = {"compiler": version, "config": args.config, "sources": len(sources), "failed": failed,
               "warnings": {f"{k} {fl}": n for (k, fl), n in by_flag.items()}, "memory": {}}
    status = 1 if failed else 0

    if not failed:
        link = cfg.options("", "cpp.linker")
        elf = out / f"{project}.elf"
        lflags = list(target)
        script = link.get("script", (None, []))[0]
        if script:
            lflags.append("-T" + str(resolve(script)))
        if link.get("systemcalls", ("",))[0].endswith("minimalimplementation"):
            lflags.append("--specs=nosys.specs")
        lflags += ["-L" + str(resolve(d)) for d in link.get("directories", (None, []))[1]]
        lflags += ["-l" + l for l in link.get("libraries", (None, []))[1]]
        lflags += ["-Wl,-Map=" + str(out / f"{project}.map"), "-Wl,--print-memory-usage"] + CUBEIDE_LINK
        objs = [str(o) for _, o, _, _ in results]
        r = subprocess.run([PREFIX + "g++", "-o", str(elf)] + objs + lflags, capture_output=True, text=True)
        print(r.stdout, end="")
        if r.returncode:
            print(r.stderr)
            summary["failed"] = ["(link)"]
            status = 1
        else:
            if r.stderr.strip():
                print(r.stderr)
            subprocess.run([PREFIX + "objcopy", "-O", "binary", str(elf), str(out / f"{project}.bin")], check=True)
            for m in re.finditer(r"^\s*(\w+):\s+(\d+)\s*(\w?B)\s+(\d+)\s*(\w?B)\s+([\d.]+)%", r.stdout, re.M):
                unit = {"B": 1, "KB": 1024, "MB": 1024 * 1024, "GB": 1024 ** 3}
                used = int(m.group(2)) * unit[m.group(3)]
                size = int(m.group(4)) * unit[m.group(5)]
                summary["memory"][m.group(1)] = {"used": used, "size": size, "percent": float(m.group(6))}
            summary["bin_bytes"] = (out / f"{project}.bin").stat().st_size
            print(f"{project}.bin: {summary['bin_bytes']} bytes")
            limits = dict(x.split("=", 1) for x in args.fail_usage)
            for region, mem in summary["memory"].items():
                if region in limits and mem["percent"] > float(limits[region]):
                    print(f"ERROR: {region} {mem['percent']:.1f}% used, limit {limits[region]}%")
                    status = 3
                elif mem["percent"] > args.warn_usage:
                    print(f"note: {region} {mem['percent']:.1f}% used")

    if args.json:
        args.json.write_text(json.dumps(summary, indent=1))
    return status


if __name__ == "__main__":
    sys.exit(main())
