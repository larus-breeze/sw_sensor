#!/usr/bin/env python3
"""Summaries of a CI build for the GitHub run page.

    ci_summary.py from-build ELF LOG --json FILE
        The summary ci_build.py --json writes, for a build that ci_build.py
        didn't do (STM32CubeIDE): memory usage per region from the ELF and the
        linker script in .cproject, compiler warnings from the build log.
    ci_summary.py markdown HEAD.json [BASE.json]
        Memory usage and warnings as Markdown, with the change against the
        PR's base if BASE.json exists.

Standard library only.
"""

import argparse
import json
import os
import re
import struct
import sys
from collections import Counter
from pathlib import Path

from ci_build import DIAG, SW_STM32_DIR, Config, resolve


def read_elf(path):
    """GCC identification (.comment) and PT_LOAD segments (paddr, vaddr, filesz, memsz)."""
    data = Path(path).read_bytes()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        sys.exit(f"{path} is not a 32-bit little-endian ELF file")
    e_phoff, e_shoff = struct.unpack_from("<II", data, 0x1C)
    e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx = struct.unpack_from("<HHHHH", data, 0x2A)
    segments = []
    for i in range(e_phnum):
        p_type, _, vaddr, paddr, filesz, memsz = struct.unpack_from("<IIIIII", data, e_phoff + i * e_phentsize)
        if p_type == 1:
            segments.append((paddr, vaddr, filesz, memsz))
    section = lambda i: struct.unpack_from("<IIIIII", data, e_shoff + i * e_shentsize)
    names = section(e_shstrndx)[4]
    compiler = None
    for i in range(e_shnum):
        name, _, _, _, offset, size = section(i)
        if data[names + name:data.index(b"\0", names + name)] == b".comment":
            compiler = next((s.decode() for s in data[offset:offset + size].split(b"\0") if s.startswith(b"GCC:")), None)
    return compiler, segments


def memory_regions(config):
    script = Config(config).options("", "cpp.linker")["script"][0]
    text = re.search(r"MEMORY\s*\{(.*?)\}", resolve(script).read_text(), re.S).group(1)
    unit = {"": 1, "K": 1024, "M": 1024 * 1024}
    return {m.group(1): (int(m.group(2), 0), int(m.group(3)) * unit[m.group(4).upper()])
            for m in re.finditer(r"(\w+)\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(\w+)\s*,\s*LENGTH\s*=\s*(\d+)([KkMm]?)", text)}


def from_build(elf, log, config):
    compiler, segments = read_elf(elf)
    regions = memory_regions(config)
    used = dict.fromkeys(regions, 0)

    # Like ld --print-memory-usage: up to the highest address used, the load
    # image where it is stored, the run-time size (including .bss) where it runs.
    for paddr, vaddr, filesz, memsz in segments:
        for addr, size in ((paddr, filesz), (vaddr, memsz)):
            for r, (origin, length) in regions.items():
                if size and origin <= addr < origin + length:
                    used[r] = max(used[r], addr + size - origin)

    text = Path(log).read_text(errors="replace")
    build_dir = SW_STM32_DIR / config
    diags = {(m.group("kind"), m.group("flag") or "(no flag)",
              os.path.relpath(os.path.normpath(build_dir / m.group("file")), SW_STM32_DIR), int(m.group("line")), m.group("msg"))
             for m in DIAG.finditer(text)}
    by_flag = Counter((k, fl) for k, fl, *_ in diags)
    return {"compiler": compiler, "config": config,
            "sources": len(re.findall(r"^arm-none-eabi-g(?:cc|\+\+) .* -c ", text, re.M)), "failed": [],
            "warnings": {f"{k} {fl}": n for (k, fl), n in by_flag.items()},
            "memory": {r: {"used": u, "size": regions[r][1], "percent": round(100 * u / regions[r][1], 2)}
                       for r, u in used.items()}}


def markdown(head, base):
    lines = [f"### STM32 firmware: {head['compiler']}\n", f"{head['sources']} sources, {len(head['failed'])} failed",
             "\n| Region | Used | Size | Usage |" + (" Change vs. base |" if base else ""),
             "|---|---:|---:|---:|" + ("---:|" if base else "")]
    for region, m in head["memory"].items():
        if not m["used"] and region not in ("FLASH", "RAM", "CCMRAM"):
            continue
        row = f"| {region} | {m['used']:,} | {m['size']:,} | {m['percent']:.1f} % |"
        if base and region in base["memory"]:
            row += f" {m['used'] - base['memory'][region]['used']:+,} B |"
        lines.append(row)
    if head["warnings"]:
        lines += ["\n| Diagnostics | Count |" + (" Base |" if base else ""), "|---|---:|" + ("---:|" if base else "")]
        for k, n in sorted(head["warnings"].items()):
            lines.append(f"| `{k}` | {n} |" + (f" {base['warnings'].get(k, 0)} |" if base else ""))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("from-build")
    b.add_argument("elf", type=Path)
    b.add_argument("log", type=Path)
    b.add_argument("--config", default="Release")
    b.add_argument("--json", type=Path, required=True)
    m = sub.add_parser("markdown")
    m.add_argument("head", type=Path)
    m.add_argument("base", type=Path, nargs="?")
    args = ap.parse_args()

    if args.cmd == "from-build":
        args.json.write_text(json.dumps(from_build(args.elf, args.log, args.config), indent=1))
    else:
        load = lambda p: json.loads(p.read_text()) if p and p.exists() else None
        head = load(args.head)
        print(markdown(head, load(args.base)) if head else "Build did not produce a summary.")


if __name__ == "__main__":
    main()
