#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/install"),
]

PATCH = {
    "package_scan_range_deg": "0.0",
    "package_scan_step_deg": "30.0",
    "package_scan_wait_sec": "0.0",
    "package_scan_timeout_sec": "3.0",
}

def is_backup(p: Path) -> bool:
    return ".bak" in p.name or p.name.endswith("~")

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_force_disable_scan_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def patch_file(p: Path) -> bool:
    if not p.exists() or is_backup(p):
        return False

    if p.suffix not in [".yaml", ".yml", ".py", ".xml"]:
        return False

    s = p.read_text(errors="ignore")
    old = s

    if not any(k in s for k in PATCH.keys()):
        return False

    for k, v in PATCH.items():
        s = re.sub(
            rf"(^\s*{re.escape(k)}\s*:\s*).*$",
            rf"\g<1>{v}",
            s,
            flags=re.MULTILINE,
        )

        # 兼容 launch/config 里可能写成 'key': value
        s = re.sub(
            rf"(['\"]{re.escape(k)}['\"]\s*:\s*)[-0-9.]+",
            rf"\g<1>{v}",
            s,
        )

    if s == old:
        return False

    backup(p)
    p.write_text(s)
    print(f"[OK] patched: {p}")
    return True

def main():
    count = 0

    for root in ROOTS:
        if not root.exists():
            continue
        for p in root.rglob("*"):
            if patch_file(p):
                count += 1

    print("")
    print("[DONE] forced package scan params:")
    for k, v in PATCH.items():
        print(f"  {k}: {v}")
    print(f"[DONE] patched files: {count}")

if __name__ == "__main__":
    main()
