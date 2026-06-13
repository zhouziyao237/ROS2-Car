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

MAX_LINEAR = 1.8
MAX_THETA = 1.8

# 加速度不要太小，不然会一卡一卡；也不要太大，避免到货架前甩头
ACC_LINEAR = 2.5
ACC_THETA = 4.0

MIN_VEL_X = -0.30
MIN_VEL_Y = -1.8
MIN_THETA = -1.8

MIN_SPEED_THETA = 0.25


def is_backup_file(p: Path) -> bool:
    n = p.name
    return (
        ".bak" in n
        or ".before_manual_rollback_" in n
        or ".broken" in n
        or n.endswith("~")
    )


def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_nav_speed_1p8_{STAMP}")
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

    speed_pairs = {
        "max_vel_x": MAX_LINEAR,
        "min_vel_x": MIN_VEL_X,

        "max_vel_y": MAX_LINEAR,
        "min_vel_y": MIN_VEL_Y,

        "max_speed_xy": MAX_LINEAR,
        "min_speed_xy": 0.0,

        "max_vel_theta": MAX_THETA,
        "min_speed_theta": MIN_SPEED_THETA,

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


def patch_recursive(obj) -> bool:
    changed = False

    if isinstance(obj, dict):
        if "controller_server" in obj and isinstance(obj["controller_server"], dict):
            params = obj["controller_server"].get("ros__parameters")
            if isinstance(params, dict):
                if patch_controller_params(params):
                    changed = True

        if "velocity_smoother" in obj and isinstance(obj["velocity_smoother"], dict):
            params = obj["velocity_smoother"].get("ros__parameters")
            if isinstance(params, dict):
                if patch_velocity_smoother(params):
                    changed = True

        if patch_controller_params(obj):
            changed = True

        if "max_velocity" in obj or "max_accel" in obj or "max_decel" in obj:
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

    if not any(k in str(data) for k in [
        "controller_server",
        "velocity_smoother",
        "FollowPath",
        "max_vel_theta",
        "max_velocity",
    ]):
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

    print("")
    print("[DONE] Nav2 max linear speed = 1.8")
    print("[DONE] Nav2 max angular speed = 1.8")
    print("[DONE] velocity_smoother max_velocity = [1.8, 1.8, 1.8]")
    print("[DONE] acc_lim_theta = 4.0")
    print(f"[DONE] patched yaml files: {count}")


if __name__ == "__main__":
    main()
