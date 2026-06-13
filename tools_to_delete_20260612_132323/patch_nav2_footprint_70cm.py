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

FOOTPRINT = '[[0.35, 0.35], [0.35, -0.35], [-0.35, -0.35], [-0.35, 0.35]]'

def backup(p: Path):
    b = p.with_suffix(p.suffix + f".bak_footprint_70cm_{STAMP}")
    shutil.copy2(p, b)
    return b

def patch_file(p: Path):
    s = p.read_text(errors="ignore")
    old = s

    # 只处理看起来是 Nav2 参数的文件
    if not any(k in s for k in ["global_costmap", "local_costmap", "controller_server", "planner_server"]):
        return False

    # footprint: "[[...]]" 或 footprint: [[...]]
    s = re.sub(
        r'footprint:\s*["\']?\[\[.*?\]\]["\']?',
        f'footprint: "{FOOTPRINT}"',
        s,
        flags=re.S,
    )

    # 如果原来用 robot_radius，也改成 0.35m
    s = re.sub(
        r'robot_radius:\s*[-0-9.]+',
        'robot_radius: 0.35',
        s,
    )

    # inflation 半径改到 0.40m，略大于 0.35m 的安全外廓内切半径
    s = re.sub(
        r'inflation_radius:\s*[-0-9.]+',
        'inflation_radius: 0.40',
        s,
    )

    # 顺便降低 Nav2 自己的最大角速度，避免恢复/局部规划时原地猛转
    s = re.sub(
        r'max_vel_theta:\s*[-0-9.]+',
        'max_vel_theta: 0.35',
        s,
    )
    s = re.sub(
        r'acc_lim_theta:\s*[-0-9.]+',
        'acc_lim_theta: 0.5',
        s,
    )
    s = re.sub(
        r'decel_lim_theta:\s*[-0-9.]+',
        'decel_lim_theta: -0.5',
        s,
    )

    if s == old:
        return False

    b = backup(p)
    p.write_text(s)
    print(f"[OK] patched {p}")
    print(f"     backup: {b}")
    return True

patched = 0

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*.yaml"):
        if patch_file(p):
            patched += 1

print("")
print(f"[DONE] patched files: {patched}")
print("Nav2 footprint is now 70cm x 70cm, inflation_radius=0.40, max_vel_theta=0.35")
