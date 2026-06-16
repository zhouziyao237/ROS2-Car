#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CANDIDATES = [
    Path("/host-home/src/target_detection/oak_package_servo/config/oak_package_servo.yaml"),
    Path("/host-home/src/target_detection/oak_package_servo/config/oak_package_servo.yml"),
]

cfg = None
for p in CANDIDATES:
    if p.exists():
        cfg = p
        break

if cfg is None:
    print("[ERROR] cannot find oak_package_servo.yaml")
    print("Try:")
    print("  find /host-home/src -name 'oak_package_servo.y*ml'")
    raise SystemExit(1)

bak = cfg.with_suffix(cfg.suffix + f".bak_standoff55_{STAMP}")
shutil.copy2(cfg, bak)
print(f"[BACKUP] {cfg} -> {bak}")

text = cfg.read_text(encoding="utf-8")

def set_key(text, key, value):
    pattern = rf"^(\s*{re.escape(key)}\s*:\s*).*$"
    repl = rf"\g<1>{value}"
    new_text, n = re.subn(pattern, repl, text, flags=re.M)
    if n == 0:
        print(f"[WARN] key not found: {key}")
    else:
        print(f"[PATCH] {key}: {value}")
    return new_text

text = set_key(text, "standoff_m", "0.55")
text = set_key(text, "min_standoff_m", "0.50")
text = set_key(text, "fine_standoff_m", "0.55")

cfg.write_text(text, encoding="utf-8")

print("")
print("[DONE] OAK package servo target distance set to 55cm.")
print("")
print("Check with:")
print(f"  grep -n \"standoff_m\\|min_standoff_m\\|fine_standoff_m\" {cfg}")
