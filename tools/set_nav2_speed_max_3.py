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

MAX_LINEAR = 3.0
MAX_THETA = 3.0

# 为了不让车突然炸出去，加速度先不要设得无限大
ACC_LINEAR = 2.5
ACC_THETA = 5.0

# 后退速度不要给到 -3.0，避免 DWB 自己倒车太猛
MIN_VEL_X = -0.3
MIN_VEL_Y = -3.0
MIN_THETA = -3.0


def is_backup_file(p: Path) -> bool:
    n = p.name
    return (
        ".bak" in n
        or ".before_manual_rollback_" in n
        or ".broken" in n
        or n.endswith("~")
    )


def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_nav_speed_3_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")
    return bak


def yaml_ok(p: Path) -> bool:
    try:
        yaml.safe_load(p.read_text(errors="ignore"))
        return True
    except Exception as e:
        print(f"[YAML ERROR] {p}: {e}")
        return False


def patch_controller_params(params: dict) -> bool:
    changed = False

    # 兼容 DWB / FollowPath 直接参数
    speed_pairs = {
        "max_vel_x": MAX_LINEAR,
        "min_vel_x": MIN_VEL_X,
        "max_vel_y": MAX_LINEAR,
        "min_vel_y": MIN_VEL_Y,
        "max_speed_xy": MAX_LINEAR,
        "min_speed_xy": 0.0,
        "max_vel_theta": MAX_THETA,
        "min_speed_theta": 0.0,

        "acc_lim_x": ACC_LINEAR,
        "acc_lim_y": ACC_LINEAR,
        "acc_lim_theta": ACC_THETA,
        "decel_lim_x": -ACC_LINEAR,
        "decel_lim_y": -ACC_LINEAR,
        "decel_lim_theta": -ACC_THETA,
    }

    for k, v in speed_pairs.items():
        if k in params and params.get(k) != v:
            params[k] = v
            changed = True

    # 常见结构：FollowPath:
    #   max_vel_x: ...
    fp = params.get("FollowPath")
    if isinstance(fp, dict):
        for k, v in speed_pairs.items():
            if fp.get(k) != v:
                fp[k] = v
                changed = True

    return changed


def patch_velocity_smoother(params: dict) -> bool:
    changed = False

    desired = {
        "max_velocity": [MAX_LINEAR, MAX_LINEAR, MAX_THETA],
        "min_velocity": [MIN_VEL_X, MIN_VEL_Y, MIN_THETA],
        "max_accel": [ACC_LINEAR, ACC_LINEAR, ACC_THETA],
        "max_decel": [-ACC_LINEAR, -ACC_LINEAR, -ACC_THETA],
    }

    for k, v in desired.items():
        if k in params and params.get(k) != v:
            params[k] = v
            changed = True

    return changed


def patch_behavior_backup(params: dict) -> bool:
    """
    recovery backup 不建议给 3.0。
    这里只把 backup 的速度从 0.05 提到 0.15，避免脱困太慢。
    """
    changed = False

    backup = params.get("backup")
    if isinstance(backup, dict):
        # 部分 Nav2 backup 参数不在这里，BT XML 里也可能写死 backup_speed。
        if "speed" in backup and backup.get("speed") != 0.15:
            backup["speed"] = 0.15
            changed = True

    return changed


def patch_nav_bt_xml(path: Path) -> bool:
    """
    把 no-spin BT XML 里的 BackUp backup_speed 从 0.05 提到 0.15。
    注意不是 3.0，backup 是 recovery 脱困动作，太快容易撞。
    """
    if not path.exists():
        return False

    s = path.read_text(errors="ignore")
    old = s

    s = s.replace('backup_speed="0.05"', 'backup_speed="0.15"')
    s = s.replace('backup_speed="0.050"', 'backup_speed="0.15"')

    if s == old:
        return False

    backup(path)
    path.write_text(s)
    print(f"[OK] patched backup_speed in {path}")
    return True


def patch_recursive(obj) -> bool:
    changed = False

    if isinstance(obj, dict):
        # controller_server:
        #   ros__parameters:
        #     FollowPath:
        #       max_vel_x...
        if "controller_server" in obj and isinstance(obj["controller_server"], dict):
            params = obj["controller_server"].get("ros__parameters")
            if isinstance(params, dict):
                if patch_controller_params(params):
                    changed = True

        # velocity_smoother:
        #   ros__parameters:
        #     max_velocity...
        if "velocity_smoother" in obj and isinstance(obj["velocity_smoother"], dict):
            params = obj["velocity_smoother"].get("ros__parameters")
            if isinstance(params, dict):
                if patch_velocity_smoother(params):
                    changed = True

        # behavior_server:
        if "behavior_server" in obj and isinstance(obj["behavior_server"], dict):
            params = obj["behavior_server"].get("ros__parameters")
            if isinstance(params, dict):
                if patch_behavior_backup(params):
                    changed = True

        # 递归兼容直接嵌套 FollowPath / max_velocity
        if patch_controller_params(obj):
            changed = True

        if "max_velocity" in obj or "max_accel" in obj:
            if patch_velocity_smoother(obj):
                changed = True

        for v in obj.values():
            if patch_recursive(v):
                changed = True

    elif isinstance(obj, list):
        for item in obj:
            if patch_recursive(item):
                changed = True

    return changed


def patch_yaml_file(path: Path):
    if not path.exists() or is_backup_file(path):
        return False

    try:
        data = yaml.safe_load(path.read_text(errors="ignore"))
    except Exception:
        return False

    if not isinstance(data, dict):
        return False

    if not any(k in data for k in ["controller_server", "velocity_smoother", "behavior_server"]):
        return False

    changed = patch_recursive(data)

    if not changed:
        return False

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
        shutil.copy2(bak, path)
        print(f"[RESTORE] invalid yaml restored: {path}")
        return False

    print(f"[OK] patched nav speed yaml: {path}")
    return True


def main():
    count = 0

    for root in ROOTS:
        if not root.exists():
            continue
        for path in list(root.rglob("*.yaml")) + list(root.rglob("*.yml")):
            if patch_yaml_file(path):
                count += 1

    # patch no-spin BT XML backup speed
    bt_files = [
        Path("/host-home/src/Navigation/robot2_bringup/config/navigate_to_pose_pickup_sentry_style.xml"),
        Path("/host-home/src/Navigation/robot2_bringup/config/navigate_through_poses_pickup_sentry_style.xml"),
        Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/navigate_to_pose_pickup_sentry_style.xml"),
        Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/navigate_through_poses_pickup_sentry_style.xml"),
    ]

    for bt in bt_files:
        patch_nav_bt_xml(bt)

    print(f"[DONE] patched yaml files: {count}")
    print(f"[DONE] Nav2 max linear speed = {MAX_LINEAR}")
    print(f"[DONE] Nav2 max angular speed = {MAX_THETA}")
    print("[DONE] Recovery backup speed = 0.15, not 3.0")


if __name__ == "__main__":
    main()
