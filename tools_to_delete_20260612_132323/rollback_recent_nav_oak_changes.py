#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import subprocess
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

# 明确回滚我们这轮改过/可能改过的关键文件
FILES = [
    # OAK package search code/config
    Path("/host-home/src/target_detection/oak_package_search/oak_package_search/oak_package_search_node.py"),
    Path("/host-home/install/oak_package_search/lib/python3.10/site-packages/oak_package_search/oak_package_search_node.py"),
    Path("/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml"),
    Path("/host-home/install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml"),

    # bringup modes
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),

    # mission BT C++ source
    Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp"),

    # semantic nav2 params
    Path("/host-home/src/Navigation/robot_logistics_semantic/config/nav2_params.yaml"),
    Path("/host-home/install/robot_logistics_semantic/share/robot_logistics_semantic/config/nav2_params.yaml"),

    # nav_stack_core params
    Path("/host-home/src/Navigation/nav_stack_core/config/nav2_params.yaml"),
    Path("/host-home/src/Navigation/nav_stack_core/config/nav2_params_min.yaml"),
    Path("/host-home/src/Navigation/nav_stack_core/config/nav2_params_localization.yaml"),
    Path("/host-home/src/Navigation/nav_stack_core/config/navigate_to_pose_no_backup.xml"),

    # nav_stack_core_v2 params
    Path("/host-home/src/Navigation/nav_stack_core_v2/config/nav2_params_min.yaml"),
    Path("/host-home/src/Navigation/nav_stack_core_v2/config/nav2_robot2_fused.yaml"),
    Path("/host-home/install/nav_stack_core_v2/share/nav_stack_core_v2/config/nav2_params_min.yaml"),
    Path("/host-home/install/nav_stack_core_v2/share/nav_stack_core_v2/config/nav2_robot2_fused.yaml"),
]

# 这些是我们这轮脚本生成的备份类型，按优先级从“最早、最接近原始”到“后面补丁前”
BACKUP_PATTERNS = [
    ".bak_oak_search_guard_*",
    ".bak_reentry_guard_*",
    ".bak_local_scan_front_*",
    ".bak_d435_frame_config_*",
    ".bak_footprint_70cm_*",
    ".bak_disable_spin_*",
    ".bak_clean_guard_yaml_*",
    ".bak_clean_guard_bringup_*",
    ".bak_clean_monkey_patch_*",
    ".bak_monkey_guard_*",
    ".bak_force_auto_before_goal_*",
    ".bak_force_auto_before_nav_*",
    ".bak_cmd_mux_axis_*",
]


def py_compiles(p: Path) -> bool:
    if p.suffix != ".py":
        return True
    r = subprocess.run(
        ["python3", "-m", "py_compile", str(p)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return r.returncode == 0


def yaml_loads(p: Path) -> bool:
    if p.suffix not in [".yaml", ".yml"]:
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
    return r.returncode == 0


def candidate_score(c: Path) -> tuple:
    name = c.name

    # 优先恢复最早的 oak_search_guard，因为这是第一次改文件前备份
    order = 999
    for i, pat in enumerate(BACKUP_PATTERNS):
        raw = pat.replace("*", "")
        if raw in name:
            order = i
            break

    # 提取时间戳排序
    m = re.search(r"(\d{8}_\d{6})", name)
    ts = m.group(1) if m else "99999999_999999"

    return (order, ts, name)


def find_backup(p: Path):
    candidates = []
    for pat in BACKUP_PATTERNS:
        candidates.extend(p.parent.glob(p.name + pat))

    # 过滤掉坏的 Python/YAML 备份
    good = []
    for c in candidates:
        if not c.exists():
            continue
        if not py_compiles(c):
            continue
        if not yaml_loads(c):
            continue
        good.append(c)

    if not good:
        return None

    good.sort(key=candidate_score)
    return good[0]


def rollback(p: Path):
    if not p.exists():
        print(f"[SKIP] missing current file: {p}")
        return False

    backup = find_backup(p)
    if backup is None:
        print(f"[NO BACKUP] {p}")
        return False

    current_save = p.with_suffix(p.suffix + f".before_manual_rollback_{STAMP}")
    shutil.copy2(p, current_save)
    shutil.copy2(backup, p)

    print(f"[ROLLBACK] {p}")
    print(f"           restored from: {backup}")
    print(f"           current saved: {current_save}")

    return True


changed = 0
for f in FILES:
    if rollback(f):
        changed += 1

print("")
print(f"[DONE] rollback changed files: {changed}")

print("")
print("[CHECK] Python compile")
for p in FILES:
    if p.exists() and p.suffix == ".py":
        ok = py_compiles(p)
        print(f"  {'OK' if ok else 'FAIL'} {p}")

print("")
print("[CHECK] YAML load")
for p in FILES:
    if p.exists() and p.suffix in [".yaml", ".yml"]:
        ok = yaml_loads(p)
        print(f"  {'OK' if ok else 'FAIL'} {p}")

print("")
print("[NEXT] If mission_btcpp_node.cpp was restored, rebuild robot_logistics_task.")
