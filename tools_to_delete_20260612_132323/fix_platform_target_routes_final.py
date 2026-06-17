#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

BRINGUP_FILES = [
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
]

MISSION_YAML = Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml")
MISSION_CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

def backup(p: Path):
    if not p.exists():
        print(f"[skip] not found: {p}")
        return False
    bak = p.with_suffix(p.suffix + f".bak_route_fix_{STAMP}")
    bak.write_text(p.read_text(encoding="utf-8"), encoding="utf-8")
    print(f"[backup] {p} -> {bak}")
    return True

def patch_bringup(p: Path):
    if not backup(p):
        return

    lines = p.read_text(encoding="utf-8").splitlines(True)
    out = []
    module = None

    for line in lines:
        m = re.match(r"^  ([A-Za-z0-9_]+):\s*$", line)
        if m:
            module = m.group(1)

        if re.match(r"^\s+platform_target_topic\s*:", line):
            indent = re.match(r"^(\s*)", line).group(1)

            if module == "chassis_serial":
                line = f"{indent}platform_target_topic: /platform/target\n"
                print(f"[patch] {p}: chassis_serial -> /platform/target")

            elif module == "mission_bt":
                line = f"{indent}platform_target_topic: /task/platform_target_legacy\n"
                print(f"[patch] {p}: mission_bt -> /task/platform_target_legacy")

        out.append(line)

    p.write_text("".join(out), encoding="utf-8")

def patch_mission_yaml():
    if not backup(MISSION_YAML):
        return

    text = MISSION_YAML.read_text(encoding="utf-8")
    text = re.sub(
        r"^(\s*)platform_target_topic\s*:.*$",
        r"\1platform_target_topic: /task/platform_target_legacy",
        text,
        flags=re.M,
    )
    MISSION_YAML.write_text(text, encoding="utf-8")
    print("[patch] mission_task.yaml mission platform_target_topic -> /task/platform_target_legacy")

def patch_mission_cpp():
    if not backup(MISSION_CPP):
        return

    text = MISSION_CPP.read_text(encoding="utf-8")
    text = text.replace(
        'declare_parameter<std::string>("platform_target_topic","/platform/target")',
        'declare_parameter<std::string>("platform_target_topic","/task/platform_target_legacy")'
    )
    MISSION_CPP.write_text(text, encoding="utf-8")
    print("[patch] mission_btcpp_node.cpp default platform_target_topic -> /task/platform_target_legacy")

def patch_rm_serial_configs():
    candidates = []
    for root in [Path("/host-home/src"), Path("/host-home/install")]:
        if root.exists():
            candidates += list(root.rglob("rm_serial_driver*/config/serial_driver.yaml"))

    seen = set()
    for p in candidates:
        if p in seen:
            continue
        seen.add(p)

        if not backup(p):
            continue

        text = p.read_text(encoding="utf-8")
        text = re.sub(
            r"^(\s*)platform_target_topic\s*:.*$",
            r"\1platform_target_topic: /platform/target",
            text,
            flags=re.M,
        )
        p.write_text(text, encoding="utf-8")
        print(f"[patch] {p}: rm_serial_driver -> /platform/target")

def main():
    for p in BRINGUP_FILES:
        patch_bringup(p)

    patch_mission_yaml()
    patch_mission_cpp()
    patch_rm_serial_configs()

    print("[done] final route:")
    print("       rm_serial_driver/chassis_serial -> /platform/target")
    print("       mission_btcpp_node              -> /task/platform_target_legacy")

if __name__ == "__main__":
    main()
