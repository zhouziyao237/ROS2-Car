#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = [
    Path("/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml"),
    Path("/host-home/install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml"),

    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
]

PARAMS = {
    # 关键固化项
    "target_mode": "front_yaw",
    "oak_yaw_offset_deg": 180.0,

    # 保留当前可用的 OAK 对正参数
    "oak_camera_frame": "oak-cam-Link",
    "fixed_frame": "map",
    "enable_motion": True,
    "require_tf_for_alignment": True,
    "align_only": True,

    "max_wz": 1.2,
    "kp_wz": 1.6,
    "min_wz": 0.12,
    "yaw_control_sign": 1.0,

    "align_yaw_tolerance_deg": 4.0,
    "scan_start_yaw_tolerance_deg": 5.0,
    "yaw_deadband_deg": 1.5,

    # 扫码阶段暂时保持保守，不直接改成很快
    "scan_range_deg": 12.0,
    "scan_speed_wz": 0.18,
    "scan_timeout_sec": 40.0,
    "max_scan_passes": 1,

    "match_only_in_scan": True,
    "ignore_detection_before_request": True,
    "min_search_time_before_match_sec": 0.3,
    "min_confirm_matches": 2,
    "datamatrix_fresh_timeout_sec": 1.2,
}


def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_solid_front_yaw_180_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")


def yaml_ok(path: Path) -> bool:
    try:
        yaml.safe_load(path.read_text(errors="ignore"))
        return True
    except Exception as e:
        print(f"[YAML ERROR] {path}: {e}")
        return False


def looks_like_oak_dict(d: dict) -> bool:
    keys = set(d.keys())
    return bool(
        "oak_camera_frame" in keys
        or "target_mode" in keys
        or "oak_yaw_offset_deg" in keys
        or "max_wz" in keys
        or "kp_wz" in keys
        or "scan_speed_wz" in keys
        or ("request_topic" in keys and "status_topic" in keys)
    )


def patch_obj(obj) -> bool:
    changed = False

    if isinstance(obj, dict):
        # oak_package_search.yaml 标准结构
        if "oak_package_search_node" in obj and isinstance(obj["oak_package_search_node"], dict):
            params = obj["oak_package_search_node"].setdefault("ros__parameters", {})
            if isinstance(params, dict):
                for k, v in PARAMS.items():
                    if params.get(k) != v:
                        params[k] = v
                        changed = True

        # bringup_modes.yaml inline 结构
        if looks_like_oak_dict(obj):
            for k, v in PARAMS.items():
                if obj.get(k) != v:
                    obj[k] = v
                    changed = True

        for v in obj.values():
            if patch_obj(v):
                changed = True

    elif isinstance(obj, list):
        for item in obj:
            if patch_obj(item):
                changed = True

    return changed


def patch_file(path: Path) -> bool:
    if not path.exists():
        print(f"[SKIP] missing: {path}")
        return False

    try:
        data = yaml.safe_load(path.read_text(errors="ignore"))
    except Exception as e:
        print(f"[SKIP] parse failed: {path}: {e}")
        return False

    if data is None:
        return False

    changed = patch_obj(data)

    if not changed:
        print(f"[NO CHANGE] {path}")
        return False

    backup(path)
    old = path.read_text(errors="ignore")

    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )

    if not yaml_ok(path):
        path.write_text(old)
        print(f"[RESTORE] invalid yaml restored: {path}")
        return False

    print(f"[OK] patched: {path}")
    return True


def main():
    count = 0
    for f in FILES:
        if patch_file(f):
            count += 1

    print("")
    print("[DONE] target_mode = front_yaw")
    print("[DONE] oak_yaw_offset_deg = 180.0")
    print("[DONE] OAK align speed remains max_wz=1.2, kp_wz=1.6, min_wz=0.12")
    print(f"[DONE] patched files: {count}")


if __name__ == "__main__":
    main()
