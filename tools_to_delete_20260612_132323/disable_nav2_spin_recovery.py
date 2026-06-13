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

def is_backup(p: Path) -> bool:
    name = p.name
    return ".bak" in name or name.endswith("~") or ".broken" in name

def backup(p: Path) -> Path:
    b = p.with_suffix(p.suffix + f".bak_disable_spin_{STAMP}")
    shutil.copy2(p, b)
    return b

def patch_xml(p: Path) -> bool:
    s = p.read_text(errors="ignore")
    old = s

    # 把所有 Spin 动作替换成 Wait，避免 recovery 原地旋转。
    s = re.sub(
        r'<Spin\b[^>]*/>',
        '<Wait wait_duration="1"/>',
        s
    )

    if s != old:
        b = backup(p)
        p.write_text(s)
        print(f"[OK XML] disabled Spin in {p}")
        print(f"         backup: {b}")
        return True

    return False

def patch_yaml(p: Path) -> bool:
    s = p.read_text(errors="ignore")
    old = s

    # 只处理 Nav2 参数相关文件
    if "behavior_server" not in s and "nav2_behaviors/Spin" not in s:
        return False

    # behavior_plugins: ["spin", "backup", "wait"] -> 去掉 spin
    s = re.sub(
        r'behavior_plugins:\s*\[([^\]]*)\]',
        lambda m: 'behavior_plugins: [' + ', '.join(
            item.strip()
            for item in m.group(1).split(',')
            if item.strip().strip('"\'') != "spin"
        ) + ']',
        s
    )

    # behavior_plugins:
    #   - spin
    #   - backup
    #   - wait
    # 删除 - spin
    s = re.sub(r'\n\s*-\s*spin\s*', '\n', s)

    # 保守处理：把 spin plugin 改成 wait plugin，避免某些 YAML 仍加载 spin server
    # 注意：如果某个 block 叫 spin:，它不再提供 Spin 行为，而是 Wait 行为。
    s = re.sub(
        r'plugin:\s*["\']?nav2_behaviors/Spin["\']?',
        'plugin: "nav2_behaviors/Wait"',
        s
    )

    if s != old:
        b = backup(p)
        p.write_text(s)
        print(f"[OK YAML] disabled Spin plugin in {p}")
        print(f"          backup: {b}")
        return True

    return False

patched = 0

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if is_backup(p):
            continue

        if p.suffix.lower() == ".xml":
            if patch_xml(p):
                patched += 1

        elif p.suffix.lower() in [".yaml", ".yml"]:
            if patch_yaml(p):
                patched += 1

print("")
print(f"[DONE] patched files: {patched}")
print("Spin recovery should now be disabled/replaced by Wait.")
