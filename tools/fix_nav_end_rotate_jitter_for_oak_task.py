#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/install"),
]

# Nav2 到预停点即可，不再强制终点朝向。
# 最后朝向交给 OAK 对货架正面。
NAV_XY_TOL = 0.32
NAV_YAW_TOL = 3.14

# WaitNavDoneBeforeOak 不要等 Nav2 尾部小速度等太久。
NAV_SETTLE_QUIET_SEC = 0.4
NAV_SETTLE_TIMEOUT_SEC = 2.0
NAV_SETTLE_SPEED_EPS = 0.12


def is_backup_file(p: Path) -> bool:
    n = p.name
    return (
        ".bak" in n
        or ".before_" in n
        or ".broken" in n
        or n.endswith("~")
    )


def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_fix_nav_end_rotate_jitter_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")


def patch_obj(obj) -> bool:
    changed = False

    if isinstance(obj, dict):
        # 1) mission_btcpp_node 的 WaitNavDoneBeforeOak 参数
        if "mission_btcpp_node" in obj and isinstance(obj["mission_btcpp_node"], dict):
            params = obj["mission_btcpp_node"].setdefault("ros__parameters", {})
            if isinstance(params, dict):
                desired = {
                    "nav_settle_quiet_sec": NAV_SETTLE_QUIET_SEC,
                    "nav_settle_timeout_sec": NAV_SETTLE_TIMEOUT_SEC,
                    "nav_settle_speed_eps": NAV_SETTLE_SPEED_EPS,
                }
                for k, v in desired.items():
                    if params.get(k) != v:
                        params[k] = v
                        changed = True

        # 2) 直接 ros__parameters 结构
        if "ros__parameters" in obj and isinstance(obj["ros__parameters"], dict):
            params = obj["ros__parameters"]

            # mission 参数
            if (
                "bt_xml" in params
                or "task_status_topic" in params
                or "nav_settle_timeout_sec" in params
            ):
                desired = {
                    "nav_settle_quiet_sec": NAV_SETTLE_QUIET_SEC,
                    "nav_settle_timeout_sec": NAV_SETTLE_TIMEOUT_SEC,
                    "nav_settle_speed_eps": NAV_SETTLE_SPEED_EPS,
                }
                for k, v in desired.items():
                    if params.get(k) != v:
                        params[k] = v
                        changed = True

            # controller_server / goal_checker 参数
            if "goal_checker" in params or "FollowPath" in params or "controller_plugins" in params:
                gc = params.get("goal_checker")
                if isinstance(gc, dict):
                    if gc.get("xy_goal_tolerance") != NAV_XY_TOL:
                        gc["xy_goal_tolerance"] = NAV_XY_TOL
                        changed = True
                    if gc.get("yaw_goal_tolerance") != NAV_YAW_TOL:
                        gc["yaw_goal_tolerance"] = NAV_YAW_TOL
                        changed = True

        # 3) 任意层级里的 goal_checker
        if "goal_checker" in obj and isinstance(obj["goal_checker"], dict):
            gc = obj["goal_checker"]
            if gc.get("xy_goal_tolerance") != NAV_XY_TOL:
                gc["xy_goal_tolerance"] = NAV_XY_TOL
                changed = True
            if gc.get("yaw_goal_tolerance") != NAV_YAW_TOL:
                gc["yaw_goal_tolerance"] = NAV_YAW_TOL
                changed = True

        # 4) 任意直接出现的 tolerance
        if "xy_goal_tolerance" in obj:
            if obj.get("xy_goal_tolerance") != NAV_XY_TOL:
                obj["xy_goal_tolerance"] = NAV_XY_TOL
                changed = True

        if "yaw_goal_tolerance" in obj:
            if obj.get("yaw_goal_tolerance") != NAV_YAW_TOL:
                obj["yaw_goal_tolerance"] = NAV_YAW_TOL
                changed = True

        # 5) 递归
        for v in obj.values():
            if patch_obj(v):
                changed = True

    elif isinstance(obj, list):
        for item in obj:
            if patch_obj(item):
                changed = True

    return changed


def patch_yaml_file(path: Path) -> bool:
    if not path.exists() or is_backup_file(path):
        return False

    try:
        text = path.read_text(errors="ignore")
        data = yaml.safe_load(text)
    except Exception:
        return False

    if not isinstance(data, dict):
        return False

    key_text = str(data)
    if not any(k in key_text for k in [
        "goal_checker",
        "yaw_goal_tolerance",
        "xy_goal_tolerance",
        "mission_btcpp_node",
        "nav_settle_timeout_sec",
    ]):
        return False

    changed = patch_obj(data)

    if not changed:
        return False

    backup(path)

    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )

    print(f"[OK] patched yaml: {path}")
    return True


def main():
    count = 0

    for root in ROOTS:
        if not root.exists():
            continue

        for path in list(root.rglob("*.yaml")) + list(root.rglob("*.yml")):
            if patch_yaml_file(path):
                count += 1

    print("")
    print("[DONE] Nav2 end rotate jitter fix applied")
    print(f"[DONE] goal_checker.xy_goal_tolerance = {NAV_XY_TOL}")
    print(f"[DONE] goal_checker.yaw_goal_tolerance = {NAV_YAW_TOL}")
    print(f"[DONE] nav_settle_quiet_sec = {NAV_SETTLE_QUIET_SEC}")
    print(f"[DONE] nav_settle_timeout_sec = {NAV_SETTLE_TIMEOUT_SEC}")
    print(f"[DONE] nav_settle_speed_eps = {NAV_SETTLE_SPEED_EPS}")
    print(f"[DONE] patched yaml files: {count}")


if __name__ == "__main__":
    main()
