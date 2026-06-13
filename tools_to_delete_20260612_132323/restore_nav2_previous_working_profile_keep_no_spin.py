#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

# 前面能跑的 nav2_params_min.yaml 风格
FOOTPRINT = "[[0.60, 0.50], [0.60, -0.50], [-0.60, -0.50], [-0.60, 0.50]]"
LOCAL_INFLATION = 0.20
GLOBAL_INFLATION = 0.20
COST_SCALING = 5.0
PLANNER_TOLERANCE = 0.80

NO_SPIN_TO_POSE = "/host-home/install/robot2_bringup/share/robot2_bringup/config/navigate_to_pose_pickup_sentry_style.xml"
NO_SPIN_THROUGH = "/host-home/install/robot2_bringup/share/robot2_bringup/config/navigate_through_poses_pickup_sentry_style.xml"

FILES = [
    Path("/host-home/src/Navigation/nav_stack_core_v2/config/nav2_params_min.yaml"),
    Path("/host-home/install/nav_stack_core_v2/share/nav_stack_core_v2/config/nav2_params_min.yaml"),

    Path("/host-home/src/Navigation/nav_stack_core_v2/config/nav2_robot2_fused.yaml"),
    Path("/host-home/install/nav_stack_core_v2/share/nav_stack_core_v2/config/nav2_robot2_fused.yaml"),

    Path("/host-home/src/Navigation/nav_stack_core/config/nav2_params_min.yaml"),
    Path("/host-home/install/nav_stack_core/share/nav_stack_core/config/nav2_params_min.yaml"),

    Path("/host-home/src/Navigation/robot_logistics_semantic/config/nav2_params.yaml"),
    Path("/host-home/install/robot_logistics_semantic/share/robot_logistics_semantic/config/nav2_params.yaml"),
]


def backup(path: Path):
    if path.exists():
        bak = path.with_suffix(path.suffix + f".bak_restore_prev_nav2_{STAMP}")
        shutil.copy2(path, bak)
        print(f"[BAK] {path} -> {bak}")
        return bak
    return None


def yaml_ok(path: Path) -> bool:
    try:
        yaml.safe_load(path.read_text(errors="ignore"))
        return True
    except Exception as e:
        print(f"[YAML ERROR] {path}: {e}")
        return False


def patch_bt_navigator(data):
    changed = False
    nav = data.get("bt_navigator", {})
    if not isinstance(nav, dict):
        return False

    params = nav.setdefault("ros__parameters", {})
    if not isinstance(params, dict):
        return False

    if params.get("default_nav_to_pose_bt_xml") != NO_SPIN_TO_POSE:
        params["default_nav_to_pose_bt_xml"] = NO_SPIN_TO_POSE
        changed = True

    if params.get("default_nav_through_poses_bt_xml") != NO_SPIN_THROUGH:
        params["default_nav_through_poses_bt_xml"] = NO_SPIN_THROUGH
        changed = True

    return changed


def patch_costmap_layer(costmap_params: dict, is_local: bool):
    changed = False

    if costmap_params.get("footprint") != FOOTPRINT:
        costmap_params["footprint"] = FOOTPRINT
        changed = True

    if "robot_radius" in costmap_params:
        del costmap_params["robot_radius"]
        changed = True

    plugins = costmap_params.get("plugins", [])
    if isinstance(plugins, list):
        pass

    # obstacle layer
    obs = costmap_params.get("obstacle_layer")
    if isinstance(obs, dict):
        if obs.get("enabled") is not True:
            obs["enabled"] = True
            changed = True

        sources = obs.get("observation_sources")
        if sources != "scan":
            obs["observation_sources"] = "scan"
            changed = True

        scan = obs.setdefault("scan", {})
        if isinstance(scan, dict):
            desired = {
                "topic": "/scan_front",
                "data_type": "LaserScan",
                "marking": True,
                "clearing": True,
                "obstacle_min_range": 0.18,
                "obstacle_max_range": 6.0,
                "raytrace_min_range": 0.1,
                "raytrace_max_range": 8.0,
                "max_obstacle_height": 2.0,
            }

            for k, v in desired.items():
                if scan.get(k) != v:
                    scan[k] = v
                    changed = True

    # inflation layer
    inf = costmap_params.get("inflation_layer")
    if isinstance(inf, dict):
        if inf.get("enabled") is not True:
            inf["enabled"] = True
            changed = True

        target_inflation = LOCAL_INFLATION if is_local else GLOBAL_INFLATION

        if inf.get("inflation_radius") != target_inflation:
            inf["inflation_radius"] = target_inflation
            changed = True

        if inf.get("cost_scaling_factor") != COST_SCALING:
            inf["cost_scaling_factor"] = COST_SCALING
            changed = True

    return changed


def patch_nav2(data):
    changed = False

    if patch_bt_navigator(data):
        changed = True

    local = data.get("local_costmap", {})
    if isinstance(local, dict):
        local2 = local.get("local_costmap", {})
        if isinstance(local2, dict):
            params = local2.get("ros__parameters", {})
            if isinstance(params, dict):
                if patch_costmap_layer(params, is_local=True):
                    changed = True

    global_cm = data.get("global_costmap", {})
    if isinstance(global_cm, dict):
        global2 = global_cm.get("global_costmap", {})
        if isinstance(global2, dict):
            params = global2.get("ros__parameters", {})
            if isinstance(params, dict):
                if patch_costmap_layer(params, is_local=False):
                    changed = True

    planner = data.get("planner_server", {})
    if isinstance(planner, dict):
        params = planner.get("ros__parameters", {})
        if isinstance(params, dict):
            grid = params.get("GridBased")
            if isinstance(grid, dict):
                if grid.get("tolerance") != PLANNER_TOLERANCE:
                    grid["tolerance"] = PLANNER_TOLERANCE
                    changed = True

    return changed


def patch_file(path: Path):
    if not path.exists():
        print(f"[SKIP] missing {path}")
        return

    try:
        data = yaml.safe_load(path.read_text(errors="ignore"))
    except Exception as e:
        print(f"[SKIP] cannot parse {path}: {e}")
        return

    if not isinstance(data, dict):
        return

    if not any(k in data for k in ["bt_navigator", "local_costmap", "global_costmap", "planner_server"]):
        return

    changed = patch_nav2(data)

    if not changed:
        print(f"[NO CHANGE] {path}")
        return

    bak = backup(path)

    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )

    if not yaml_ok(path):
        if bak:
            shutil.copy2(bak, path)
            print(f"[RESTORE] invalid yaml restored: {path}")
        return

    print(f"[OK] patched {path}")


def main():
    for path in FILES:
        patch_file(path)

    print("[DONE] restored previous working Nav2 motion profile, keeping no-spin BT.")
    print(f"[DONE] footprint={FOOTPRINT}")
    print(f"[DONE] inflation={LOCAL_INFLATION}")
    print(f"[DONE] planner tolerance={PLANNER_TOLERANCE}")
    print(f"[DONE] no-spin BT={NO_SPIN_TO_POSE}")


if __name__ == "__main__":
    main()
