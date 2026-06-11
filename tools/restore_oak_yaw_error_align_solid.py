#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import yaml
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

YAML_FILES = [
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),

    Path("/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml"),
    Path("/host-home/install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml"),

    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config/mission_task.yaml"),
]

PICKUP_INPUT = Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")

# 回到原先“target_yaw - current_oak_yaw”的差值闭环方案
OAK_PACKAGE_SEARCH_PARAMS = {
    "target_mode": "point_or_front_yaw",
    "oak_camera_frame": "oak-cam-Link",
    "fixed_frame": "map",

    "enable_motion": True,
    "require_tf_for_alignment": True,
    "align_only": True,

    # 你现在要固化的旋转速度
    "max_wz": 1.2,
    "kp_wz": 1.6,
    "min_wz": 0.12,
    "yaw_control_sign": 1.0,

    # 对正容差
    "align_yaw_tolerance_deg": 4.0,
    "scan_start_yaw_tolerance_deg": 5.0,
    "yaw_deadband_deg": 1.5,

    # OAK 安装角偏移先保持 0
    # 如果后面发现差 90° / 180°，只改这个参数，不再改算法
    "oak_yaw_offset_deg": 0.0,

    # 扫码阶段不要太快，太快 DataMatrix 会糊
    "scan_range_deg": 12.0,
    "scan_speed_wz": 0.18,
    "scan_timeout_sec": 40.0,
    "max_scan_passes": 1,

    "match_only_in_scan": True,
    "ignore_detection_before_request": True,
    "min_search_time_before_match_sec": 0.3,
    "min_confirm_matches": 2,
    "datamatrix_fresh_timeout_sec": 1.2,

    # 如果当前代码没 declare，这些不会生效；写着不影响
    # 这里设 0 是为了避免“同一货架短时间不重新对正”
    "same_shelf_skip_sec": 0.0,
    "search_already_ok_deg": 4.0,
    "search_start_align_deg": 6.0,
    "search_align_only": True,
}

MISSION_PARAMS = {
    "oak_frame": "oak-cam-Link",

    # mission 内部如果也有 OAK 对正参数，也同步成这套
    "oak_align_max_wz": 1.2,
    "oak_align_kp": 1.6,
    "oak_align_min_wz": 0.12,
    "oak_align_yaw_tolerance_deg": 4.0,
    "oak_align_timeout_sec": 25.0,

    # package_search 等待时间加长，避免转还没完成就 timeout
    "package_search_timeout_sec": 45.0,

    # 后面的扫码阶段不要每一步停太久
    "package_scan_range_deg": 12.0,
    "package_scan_step_deg": 2.0,
    "package_scan_wait_sec": 0.0,
    "package_scan_timeout_sec": 40.0,
}


def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_restore_oak_yaw_error_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")
    return bak


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
        or "max_wz" in keys
        or "kp_wz" in keys
        or "target_mode" in keys
        or "scan_speed_wz" in keys
        or ("request_topic" in keys and "status_topic" in keys)
    )


def looks_like_mission_dict(d: dict) -> bool:
    keys = set(d.keys())
    return bool(
        "package_search_timeout_sec" in keys
        or "oak_align_max_wz" in keys
        or "package_scan_range_deg" in keys
        or "oak_frame" in keys
        or "bt_xml" in keys
    )


def patch_yaml_obj(obj) -> bool:
    changed = False

    if isinstance(obj, dict):
        # oak_package_search.yaml 标准结构
        if "oak_package_search_node" in obj and isinstance(obj["oak_package_search_node"], dict):
            params = obj["oak_package_search_node"].setdefault("ros__parameters", {})
            if isinstance(params, dict):
                for k, v in OAK_PACKAGE_SEARCH_PARAMS.items():
                    if params.get(k) != v:
                        params[k] = v
                        changed = True

        # mission_task.yaml 标准结构
        if "mission_btcpp_node" in obj and isinstance(obj["mission_btcpp_node"], dict):
            params = obj["mission_btcpp_node"].setdefault("ros__parameters", {})
            if isinstance(params, dict):
                for k, v in MISSION_PARAMS.items():
                    if params.get(k) != v:
                        params[k] = v
                        changed = True

        # bringup_modes.yaml 里的 inline 参数
        if looks_like_oak_dict(obj):
            for k, v in OAK_PACKAGE_SEARCH_PARAMS.items():
                if obj.get(k) != v:
                    obj[k] = v
                    changed = True

        if looks_like_mission_dict(obj):
            for k, v in MISSION_PARAMS.items():
                if obj.get(k) != v:
                    obj[k] = v
                    changed = True

        for v in obj.values():
            if patch_yaml_obj(v):
                changed = True

    elif isinstance(obj, list):
        for item in obj:
            if patch_yaml_obj(item):
                changed = True

    return changed


def patch_yaml_file(path: Path) -> bool:
    if not path.exists():
        print(f"[SKIP] missing {path}")
        return False

    try:
        data = yaml.safe_load(path.read_text(errors="ignore"))
    except Exception as e:
        print(f"[SKIP] cannot parse {path}: {e}")
        return False

    if data is None:
        return False

    changed = patch_yaml_obj(data)

    if not changed:
        print(f"[NO CHANGE] {path}")
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

    print(f"[OK] patched YAML: {path}")
    return True


def patch_pickup_code_input(path: Path) -> bool:
    """
    修 stop 不自动重置的问题：
    每次输入新取件码发布 /task/shelf_goal 前，先发布 /control_mode='auto'。
    这样上一轮任务失败进入 stop_mode 后，下一次任务会自动恢复 auto。
    """
    if not path.exists():
        print(f"[SKIP] missing pickup input: {path}")
        return False

    s = path.read_text(errors="ignore")
    old = s

    if "AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611" in s:
        print(f"[NO CHANGE] pickup input already patched: {path}")
        return False

    # 找 /task/shelf_goal publisher 变量名
    pub_match = re.search(
        r'(?m)^(\s*)(self\.(\w+)\s*=\s*self\.create_publisher\(\s*String\s*,\s*[\'"]/task/shelf_goal[\'"]\s*,\s*\d+\s*\).*)$',
        s
    )

    pub_var = None

    if pub_match:
        indent = pub_match.group(1)
        pub_var = pub_match.group(3)
        insert = (
            f"\n{indent}# AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611\n"
            f"{indent}self.control_mode_pub = self.create_publisher(String, '/control_mode', 10)"
        )
        s = s[:pub_match.end(2)] + insert + s[pub_match.end(2):]
    else:
        print("[WARN] cannot find /task/shelf_goal publisher in pickup_code_input.py")
        return False

    # 在 self.<pub_var>.publish(msg) 前插入 auto
    publish_pattern = rf'(?m)^(\s*)self\.{re.escape(pub_var)}\.publish\(msg\)'
    publish_match = re.search(publish_pattern, s)

    if publish_match:
        indent = publish_match.group(1)
        auto_code = (
            f"{indent}# AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611\n"
            f"{indent}try:\n"
            f"{indent}    auto_msg = String()\n"
            f"{indent}    auto_msg.data = 'auto'\n"
            f"{indent}    self.control_mode_pub.publish(auto_msg)\n"
            f"{indent}except Exception:\n"
            f"{indent}    pass\n"
        )
        s = s[:publish_match.start()] + auto_code + s[publish_match.start():]
    else:
        print(f"[WARN] cannot find self.{pub_var}.publish(msg)")
        return False

    if s == old:
        print(f"[NO CHANGE] pickup input: {path}")
        return False

    backup(path)
    path.write_text(s)
    print(f"[OK] patched pickup_code_input auto reset: {path}")
    return True


def main():
    yaml_count = 0

    for path in YAML_FILES:
        if patch_yaml_file(path):
            yaml_count += 1

    pickup_changed = patch_pickup_code_input(PICKUP_INPUT)

    print("")
    print("[DONE] restore original yaw-error alignment profile")
    print("[DONE] OAK target_mode = point_or_front_yaw")
    print("[DONE] OAK max_wz = 1.2")
    print("[DONE] OAK kp_wz = 1.6")
    print("[DONE] OAK min_wz = 0.12")
    print("[DONE] OAK scan_speed_wz = 0.18")
    print("[DONE] mission package_search_timeout_sec = 45.0")
    print(f"[DONE] YAML patched count = {yaml_count}")
    print(f"[DONE] pickup input auto reset patched = {pickup_changed}")


if __name__ == "__main__":
    main()
