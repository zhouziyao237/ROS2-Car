#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

files = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp"),
]

for p in files:
    if not p.exists():
        print(f"[skip] not found: {p}")
        continue

    text = p.read_text(encoding="utf-8")
    bak = p.with_suffix(p.suffix + f".bak_fix_platform_topic_{STAMP}")
    bak.write_text(text, encoding="utf-8")

    new = text.replace(
        'platform_target_topic: /platform/target',
        'platform_target_topic: /task/platform_target_legacy'
    )

    new = new.replace(
        'declare_parameter<std::string>("platform_target_topic","/platform/target")',
        'declare_parameter<std::string>("platform_target_topic","/task/platform_target_legacy")'
    )

    if new != text:
        p.write_text(new, encoding="utf-8")
        print(f"[patched] {p}")
    else:
        print(f"[nochange] {p}")

print("[done] /platform/target is reserved for auto_aim_interfaces/msg/PlatformTarget only")
