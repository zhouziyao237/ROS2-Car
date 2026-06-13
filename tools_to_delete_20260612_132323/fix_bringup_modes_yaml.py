#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import subprocess
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = [
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
]

BAD_KEYS = [
    "search_already_ok_deg",
    "search_start_align_deg",
    "same_shelf_skip_sec",
    "search_align_only",
]


def yaml_ok(p: Path) -> bool:
    code = f"""
import yaml
from pathlib import Path
p = Path({str(p)!r})
with p.open('r', encoding='utf-8') as f:
    yaml.safe_load(f)
"""
    r = subprocess.run(["python3", "-c", code], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return r.returncode == 0


def restore_from_good_backup(p: Path):
    broken_backup = p.with_suffix(p.suffix + f".broken_yaml_{STAMP}")
    shutil.copy2(p, broken_backup)
    print(f"[BACKUP BROKEN YAML] {p} -> {broken_backup}")

    candidates = []
    candidates += sorted(p.parent.glob(p.name + ".bak_oak_search_guard_*"))
    candidates += sorted(p.parent.glob(p.name + ".bak_clean_guard_bringup_*"))
    candidates += sorted(p.parent.glob(p.name + ".bak_monkey_guard_*"))

    good = []
    for c in candidates:
        if yaml_ok(c):
            good.append(c)

    if not good:
        raise SystemExit(f"[ERROR] no valid YAML backup found for {p}")

    # 优先用最早的可用备份，一般是第一次乱插参数前的版本
    chosen = good[0]
    shutil.copy2(chosen, p)
    print(f"[RESTORE YAML] {p} <- {chosen}")


def patch_oak_block(p: Path):
    lines = p.read_text(errors="ignore").splitlines(True)

    # 删除之前错误插入的 guard inline 参数，因为 Python monkey patch 已经会 declare 默认值
    cleaned = []
    for line in lines:
        stripped = line.strip()
        if any(stripped.startswith(k + ":") for k in BAD_KEYS):
            print(f"[REMOVE BAD INLINE PARAM] {p}: {stripped}")
            continue
        cleaned.append(line)

    lines = cleaned

    # 只在 oak_package_search block 内改已有参数，避免全局乱改
    out = []
    in_block = False
    block_indent = None

    for line in lines:
        stripped = line.strip()
        leading = len(line) - len(line.lstrip(" "))

        if re.match(r"^\s*oak_package_search:\s*$", line):
            in_block = True
            block_indent = leading
            out.append(line)
            continue

        if in_block and stripped and leading <= block_indent and not re.match(r"^\s*-\s", line):
            in_block = False
            block_indent = None

        if in_block:
            line = re.sub(r"(\bmax_wz:\s*)[-0-9.]+", r"\g<1>0.10", line)
            line = re.sub(r"(\bmin_wz:\s*)[-0-9.]+", r"\g<1>0.03", line)
            line = re.sub(r"(\bkp_wz:\s*)[-0-9.]+", r"\g<1>0.45", line)
            line = re.sub(r"(\balign_yaw_tolerance_deg:\s*)[-0-9.]+", r"\g<1>5.0", line)

        out.append(line)

    backup = p.with_suffix(p.suffix + f".before_yaml_safe_patch_{STAMP}")
    shutil.copy2(p, backup)
    p.write_text("".join(out))
    print(f"[PATCH SAFE YAML] {p}")
    print(f"                  backup: {backup}")

    if not yaml_ok(p):
        raise SystemExit(f"[ERROR] YAML still invalid after patch: {p}")

    print(f"[YAML OK] {p}")


for p in FILES:
    if not p.exists():
        print(f"[SKIP] not found: {p}")
        continue

    if not yaml_ok(p):
        restore_from_good_backup(p)

    patch_oak_block(p)

print("")
print("[DONE] bringup_modes.yaml fixed.")
