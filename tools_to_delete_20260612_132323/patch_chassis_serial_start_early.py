#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = [
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
]

for p in FILES:
    if not p.exists():
        print(f"[SKIP] not found: {p}")
        continue

    s = p.read_text(errors="ignore")
    old = s

    # 只在 chassis_serial block 附近把 32.0 这种延迟改成 7.0
    lines = s.splitlines(True)
    out = []
    in_block = False
    block_indent = None

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))

        if re.match(r"^\s*chassis_serial:\s*$", line):
            in_block = True
            block_indent = indent
            out.append(line)
            continue

        if in_block and stripped and indent <= block_indent and not stripped.startswith("#"):
            in_block = False
            block_indent = None

        if in_block:
            line = re.sub(r"(start_delay:\s*)[-0-9.]+", r"\g<1>7.0", line)
            line = re.sub(r"(delay:\s*)32\.0", r"\g<1>7.0", line)

        out.append(line)

    s2 = "".join(out)

    if s2 != old:
        bak = p.with_suffix(p.suffix + f".bak_chassis_start_early_{STAMP}")
        shutil.copy2(p, bak)
        p.write_text(s2)
        print(f"[OK] patched {p}")
        print(f"     backup: {bak}")
    else:
        print(f"[NO CHANGE] {p}")

print("[DONE]")
