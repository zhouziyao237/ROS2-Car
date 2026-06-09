#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import shutil
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

XML_DIRS = [
    Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree"),
    Path("/host-home/src/robot_logistics_task/behavior_tree"),
]

changed_any = False

for d in XML_DIRS:
    if not d.exists():
        continue

    for p in sorted(d.glob("*.xml")):
        s = p.read_text(errors="ignore")
        old = s

        # 删除 StopBeforeOakAlign 节点本身
        s = re.sub(r'\s*<StopBeforeOakAlign\s*/>\s*', '\n      ', s)

        # 删除可能残留的相关注释
        s = re.sub(r'\s*<!--[^>]*StopBeforeOakAlign[^>]*-->\s*', '\n      ', s)
        s = re.sub(r'\s*<!--[^>]*OAK[^>]*强制停[^>]*-->\s*', '\n      ', s)
        s = re.sub(r'\s*<!--[^>]*停止[^>]*OAK[^>]*-->\s*', '\n      ', s)

        if s != old:
            bak = p.with_suffix(p.suffix + f".bak_remove_stop_before_oak_{STAMP}")
            shutil.copy2(p, bak)
            p.write_text(s)
            changed_any = True
            print(f"[OK] removed StopBeforeOakAlign from {p}")
            print(f"[BACKUP] {bak}")
        else:
            print(f"[SKIP] no StopBeforeOakAlign in {p}")

if not changed_any:
    print("[INFO] no XML needed change")

print("[DONE]")
