#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
import subprocess
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/install"),
]

FOOTPRINT = 'footprint: "[[0.35, 0.35], [0.35, -0.35], [-0.35, -0.35], [-0.35, 0.35]]"'
ROBOT_RADIUS = "robot_radius: 0.35"
INFLATION_RADIUS = "inflation_radius: 0.60"


def is_backup(p: Path) -> bool:
    name = p.name
    return (
        ".bak" in name
        or name.endswith("~")
        or ".before_manual_rollback_" in name
        or ".broken" in name
    )


def backup(p: Path):
    b = p.with_suffix(p.suffix + f".bak_no_spin_footprint_{STAMP}")
    shutil.copy2(p, b)
    return b


def yaml_ok(p: Path) -> bool:
    if p.suffix.lower() not in [".yaml", ".yml"]:
        return True

    code = f"""
import yaml
with open({str(p)!r}, 'r', encoding='utf-8') as f:
    yaml.safe_load(f)
"""
    r = subprocess.run(
        ["python3", "-c", code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    if r.returncode != 0:
        print(f"[YAML FAIL] {p}")
        print(r.stderr)
        return False

    return True


def patch_yaml(p: Path) -> bool:
    s = p.read_text(errors="ignore")
    old = s

    # 只处理可能是 Nav2 参数的 YAML
    nav2_keywords = [
        "behavior_server",
        "bt_navigator",
        "controller_server",
        "planner_server",
        "local_costmap",
        "global_costmap",
        "nav2_behaviors/Spin",
        "inflation_radius",
        "footprint",
    ]

    if not any(k in s for k in nav2_keywords):
        return False

    # 1) behavior_plugins: ["spin", "backup", "wait"] 这种 inline 写法，去掉 spin
    def patch_behavior_inline(m):
        items_raw = m.group(1)
        items = []
        for item in items_raw.split(","):
            clean = item.strip().strip('"').strip("'")
            if clean and clean != "spin":
                items.append(clean)

        if not items:
            items = ["wait"]

        rendered = ", ".join([f'"{x}"' for x in items])
        return f"behavior_plugins: [{rendered}]"

    s = re.sub(
        r"behavior_plugins:\s*\[([^\]]*)\]",
        patch_behavior_inline,
        s,
    )

    # 2) behavior_plugins:
    #      - spin
    #      - backup
    #      - wait
    # 删除 - spin
    s = re.sub(
        r"(\n\s*)-\s*spin\s*(?=\n)",
        r"\1",
        s,
    )

    # 3) 如果还有 spin plugin block，改成 Wait，避免 behavior_server 继续加载 nav2_behaviors/Spin
    s = re.sub(
        r'plugin:\s*["\']?nav2_behaviors/Spin["\']?',
        'plugin: "nav2_behaviors/Wait"',
        s,
    )

    # 4) 修 footprint。你车体 60cm，按 70cm 安全范围。
    s = re.sub(
        r'^(\s*)footprint:\s*.*$',
        lambda m: m.group(1) + FOOTPRINT,
        s,
        flags=re.MULTILINE,
    )

    # 5) 修 robot_radius。如果有 robot_radius，也统一成 0.35。
    s = re.sub(
        r'^(\s*)robot_radius:\s*[-0-9.]+.*$',
        lambda m: m.group(1) + ROBOT_RADIUS,
        s,
        flags=re.MULTILINE,
    )

    # 6) 修 inflation_radius。之前日志是 0.200 < 0.510，所以至少要大于 0.510。
    s = re.sub(
        r'^(\s*)inflation_radius:\s*[-0-9.]+.*$',
        lambda m: m.group(1) + INFLATION_RADIUS,
        s,
        flags=re.MULTILINE,
    )

    if s == old:
        return False

    b = backup(p)
    p.write_text(s)

    if not yaml_ok(p):
        shutil.copy2(b, p)
        print(f"[RESTORE] YAML invalid, restored {p}")
        return False

    print(f"[OK YAML] {p}")
    print(f"          backup: {b}")
    return True


def patch_xml(p: Path) -> bool:
    s = p.read_text(errors="ignore")
    old = s

    if "<Spin" not in s and "Spin " not in s:
        return False

    # 把 Nav2 BT 里的 Spin 动作换成 Wait。这样 recovery 只等一下，不原地转。
    s = re.sub(
        r"<Spin\b[^>]*/>",
        '<Wait wait_duration="1"/>',
        s,
    )

    if s == old:
        return False

    b = backup(p)
    p.write_text(s)

    print(f"[OK XML] {p}")
    print(f"         backup: {b}")
    return True


patched = 0

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if is_backup(p):
            continue

        suffix = p.suffix.lower()

        if suffix in [".yaml", ".yml"]:
            if patch_yaml(p):
                patched += 1

        elif suffix == ".xml":
            if patch_xml(p):
                patched += 1

print("")
print(f"[DONE] patched files: {patched}")
print("")
print("[VERIFY COMMANDS]")
print('grep -R -n "nav2_behaviors/Spin\\|<Spin\\|spin_dist" /host-home/src/Navigation /host-home/install 2>/dev/null | grep -v ".bak" | head -80')
print('grep -R -n "inflation_radius:\\|footprint:\\|robot_radius:" /host-home/install/robot_logistics_semantic /host-home/install/nav_stack_core /host-home/install/nav_stack_core_v2 2>/dev/null | head -120')
