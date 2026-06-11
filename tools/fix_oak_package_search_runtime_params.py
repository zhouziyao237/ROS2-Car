#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = [
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml"),
    Path("/host-home/install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml"),
]

OAK_PARAMS = {
    "request_topic": "/target_detection/package_search/request",
    "status_topic": "/target_detection/package_search/status",
    "datamatrix_topic": "/oak/datamatrix_3d_result_json",
    "semantic_shelf_topic": "/semantic/current_shelf",
    "cmd_vel_topic": "/cmd_vel",

    # 关键：实际 URDF/TF 里是 oak-cam-Link
    "oak_camera_frame": "oak-cam-Link",
    "fixed_frame": "map",

    # 关键：OAK 面向货架允许真实运动
    "enable_motion": True,
    "require_tf_for_alignment": True,
    "align_only": True,
    "target_mode": "point_or_front_yaw",

    # 关键：降低 OAK 对正旋转速度
    "max_wz": 0.10,
    "min_wz": 0.04,
    "kp_wz": 0.8,
    "yaw_control_sign": 1.0,

    # 小角度对正阈值
    "align_yaw_tolerance_deg": 5.0,
    "scan_start_yaw_tolerance_deg": 5.0,
    "yaw_deadband_deg": 2.0,

    # 扫码阶段小范围扫
    "scan_range_deg": 12.0,
    "scan_speed_wz": 0.06,
    "scan_timeout_sec": 25.0,
    "max_scan_passes": 1,

    # DataMatrix 匹配
    "match_only_in_scan": True,
    "ignore_detection_before_request": True,
    "min_search_time_before_match_sec": 0.5,
    "min_confirm_matches": 2,
    "datamatrix_fresh_timeout_sec": 1.2,

    # guard 参数：如果当前代码支持，就会生效
    "search_already_ok_deg": 8.0,
    "search_start_align_deg": 12.0,
    "same_shelf_skip_sec": 12.0,
    "search_align_only": True,
}


def backup(path: Path):
    if path.exists():
        bak = path.with_suffix(path.suffix + f".bak_oak_runtime_params_{STAMP}")
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


def looks_like_oak_param_dict(d: dict) -> bool:
    keys = set(d.keys())
    return bool(
        "oak_camera_frame" in keys
        or "max_wz" in keys
        or "target_mode" in keys
        or "align_only" in keys
        or "request_topic" in keys and "status_topic" in keys
    )


def patch_dict_recursive(obj) -> bool:
    changed = False

    if isinstance(obj, dict):
        # 标准 oak_package_search.yaml:
        # oak_package_search_node:
        #   ros__parameters:
        #     ...
        if "oak_package_search_node" in obj and isinstance(obj["oak_package_search_node"], dict):
            node = obj["oak_package_search_node"]
            params = node.setdefault("ros__parameters", {})
            if isinstance(params, dict):
                for k, v in OAK_PARAMS.items():
                    if params.get(k) != v:
                        params[k] = v
                        changed = True

        # bringup_modes.yaml 里的参数 dict：
        # - request_topic: ...
        #   max_wz: 0.16
        if looks_like_oak_param_dict(obj):
            for k, v in OAK_PARAMS.items():
                if obj.get(k) != v:
                    obj[k] = v
                    changed = True

        for v in obj.values():
            if patch_dict_recursive(v):
                changed = True

    elif isinstance(obj, list):
        for item in obj:
            if patch_dict_recursive(item):
                changed = True

    return changed


def main():
    for path in FILES:
        if not path.exists():
            print(f"[SKIP] missing: {path}")
            continue

        data = yaml.safe_load(path.read_text(errors="ignore"))
        if data is None:
            print(f"[SKIP] empty yaml: {path}")
            continue

        changed = patch_dict_recursive(data)

        if not changed:
            print(f"[NO CHANGE] {path}")
            continue

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
                print(f"[RESTORE] restored invalid yaml: {path}")
            continue

        print(f"[OK] patched: {path}")

    print("[DONE] OAK runtime params patched.")


if __name__ == "__main__":
    main()
