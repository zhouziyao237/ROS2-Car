#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import shutil
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
BRINGUP = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")
MISSION_YAML = Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml")
MISSION_LAUNCH = Path("/host-home/src/Navigation/robot_logistics_task/launch/mission_task.launch.py")


def backup(p: Path):
    if p.exists():
        b = p.with_suffix(p.suffix + f".bak_oak_req_pose_{STAMP}")
        shutil.copy2(p, b)
        print(f"[BACKUP] {p} -> {b}")


def patch_cpp():
    if not CPP.exists():
        raise SystemExit(f"[ERROR] not found: {CPP}")

    backup(CPP)
    s = CPP.read_text(errors="ignore")
    changed = False

    # 1. Add full current_shelf JSON cache.
    if "latest_current_shelf_json" not in s:
        s = s.replace(
            "latest_current_shelf,latest_platform_status",
            "latest_current_shelf,latest_current_shelf_json,latest_platform_status",
            1,
        )
        changed = True
        print("[OK] added latest_current_shelf_json field")

    # 2. Store raw /semantic/current_shelf JSON while keeping latest_current_shelf as shelf_id.
    old_cb = (
        'current_shelf_sub_=create_subscription<std_msgs::msg::String>'
        '(get_parameter("current_shelf_topic").as_string(),10,'
        '[this](std_msgs::msg::String::SharedPtr m){'
        'std::lock_guard<std::mutex>l(ctx_->mutex);'
        'std::string shelf;'
        'if(get_string_json(m->data,"shelf_id",shelf))ctx_->latest_current_shelf=shelf;'
        'else ctx_->latest_current_shelf=m->data;'
        '});'
    )

    new_cb = (
        'current_shelf_sub_=create_subscription<std_msgs::msg::String>'
        '(get_parameter("current_shelf_topic").as_string(),10,'
        '[this](std_msgs::msg::String::SharedPtr m){'
        'std::lock_guard<std::mutex>l(ctx_->mutex);'
        'ctx_->latest_current_shelf_json=m->data;'
        'std::string shelf;'
        'if(get_string_json(m->data,"shelf_id",shelf))ctx_->latest_current_shelf=shelf;'
        'else if(get_string_json(m->data,"id",shelf))ctx_->latest_current_shelf=shelf;'
        'else ctx_->latest_current_shelf=m->data;'
        '});'
    )

    if old_cb in s:
        s = s.replace(old_cb, new_cb, 1)
        changed = True
        print("[OK] patched current_shelf callback to keep full JSON")
    elif "latest_current_shelf_json=m->data" in s:
        print("[SKIP] current_shelf callback already keeps full JSON")
    else:
        print("[WARN] current_shelf callback exact pattern not found; will try loose patch")
        s2 = s.replace(
            'std::string shelf;if(get_string_json(m->data,"shelf_id",shelf))ctx_->latest_current_shelf=shelf;else ctx_->latest_current_shelf=m->data;',
            'ctx_->latest_current_shelf_json=m->data;std::string shelf;if(get_string_json(m->data,"shelf_id",shelf))ctx_->latest_current_shelf=shelf;else if(get_string_json(m->data,"id",shelf))ctx_->latest_current_shelf=shelf;else ctx_->latest_current_shelf=m->data;',
            1,
        )
        if s2 != s:
            s = s2
            changed = True
            print("[OK] loose patched current_shelf callback")
        else:
            print("[WARN] loose patch also failed")

    # 3. Replace request JSON construction in RequestOakPackageSearchAlign to include shelf pose.
    old_req = (
        'req = "{\\"task_type\\":\\"package_search\\""'
        '\n                  ",\\"pickup_code\\":\\"" + pickup + "\\""'
        '\n                  ",\\"shelf_id\\":\\"" + ctx->task.shelf_id + "\\""'
        '\n                  ",\\"layer_id\\":\\"" + ctx->task.layer_id + "\\""'
        '\n                  ",\\"package_code\\":\\"" + ctx->task.package_code + "\\""'
        '\n                  ",\\"source\\":\\"mission_btcpp_node\\"}";'
    )

    new_req = (
        'std::string shelf_pose_extra;'
        '\n            {'
        '\n                double sx=0.0, sy=0.0, syaw=0.0;'
        '\n                bool has_x = get_number_json(ctx->latest_current_shelf_json, "x", sx);'
        '\n                bool has_y = get_number_json(ctx->latest_current_shelf_json, "y", sy);'
        '\n                bool has_yaw = get_number_json(ctx->latest_current_shelf_json, "shelf_front_yaw", syaw);'
        '\n                if(!has_yaw) has_yaw = get_number_json(ctx->latest_current_shelf_json, "yaw", syaw);'
        '\n                if(has_x && has_y){'
        '\n                    std::stringstream ps;'
        '\n                    ps << ",\\"x\\":" << sx << ",\\"y\\":" << sy;'
        '\n                    if(has_yaw){'
        '\n                        ps << ",\\"yaw\\":" << syaw << ",\\"shelf_front_yaw\\":" << syaw;'
        '\n                    }'
        '\n                    shelf_pose_extra = ps.str();'
        '\n                }'
        '\n            }'
        '\n'
        '\n            req = "{\\"task_type\\":\\"package_search\\""'
        '\n                  ",\\"pickup_code\\":\\"" + pickup + "\\""'
        '\n                  ",\\"shelf_id\\":\\"" + ctx->task.shelf_id + "\\""'
        '\n                  ",\\"layer_id\\":\\"" + ctx->task.layer_id + "\\""'
        '\n                  ",\\"package_code\\":\\"" + ctx->task.package_code + "\\""'
        '\n                  + shelf_pose_extra +'
        '\n                  ",\\"source\\":\\"mission_btcpp_node\\"}";'
    )

    if old_req in s:
        s = s.replace(old_req, new_req, 1)
        changed = True
        print("[OK] patched RequestOakPackageSearchAlign request payload with shelf x/y/yaw")
    elif "shelf_pose_extra" in s:
        print("[SKIP] request payload already includes shelf_pose_extra")
    else:
        print("[WARN] exact request block not found; trying regex patch")
        pattern = re.compile(
            r'req = "\{\\"task_type\\":\\"package_search\\"".*?'
            r'",\\"source\\":\\"mission_btcpp_node\\"\}";',
            re.S,
        )
        s2, n = pattern.subn(new_req, s, count=1)
        if n:
            s = s2
            changed = True
            print("[OK] regex patched request payload")
        else:
            print("[ERROR] could not patch request payload automatically")

    if changed:
        CPP.write_text(s)
        print(f"[OK] wrote {CPP}")
    else:
        print("[SKIP] no C++ changes written")


def patch_bringup_yaml():
    if not BRINGUP.exists():
        print(f"[SKIP] not found: {BRINGUP}")
        return

    backup(BRINGUP)
    s = BRINGUP.read_text(errors="ignore")

    # Only replace within oak_package_search block roughly.
    before = s

    # Make the current alignment test deterministic.
    s = s.replace("      align_only: false", "      align_only: true")
    s = s.replace("      enable_motion: false", "      enable_motion: true")

    # Ensure robust values exist in the block.
    if "      use_semantic_shelf_topic: true" not in s:
        s = s.replace("      target_mode: point_or_front_yaw", "      target_mode: point_or_front_yaw\n      use_semantic_shelf_topic: true")
    if "      oak_camera_frame: oak-cam-Link" not in s:
        s = s.replace("      oak_camera_frame: oak_camera_link", "      oak_camera_frame: oak-cam-Link")
    if "      package_search_request_topic" in s:
        pass

    if s != before:
        BRINGUP.write_text(s)
        print("[OK] patched bringup_modes.yaml: align_only=true, enable_motion=true")
    else:
        print("[SKIP] bringup_modes.yaml already suitable or pattern not found")


def patch_mission_timeout():
    for p in [MISSION_YAML, MISSION_LAUNCH]:
        if not p.exists():
            continue
        backup(p)
        s = p.read_text(errors="ignore")
        before = s
        s = s.replace("package_search_timeout_sec: 20.0", "package_search_timeout_sec: 60.0")
        s = s.replace("'package_search_timeout_sec': 20.0", "'package_search_timeout_sec': 60.0")
        if s != before:
            p.write_text(s)
            print(f"[OK] patched timeout to 60s: {p}")
        else:
            print(f"[SKIP] timeout already patched or pattern not found: {p}")


def main():
    print("============================================================")
    print("[PATCH] BT request carries semantic shelf pose to oak_package_search")
    print("============================================================")
    patch_cpp()
    patch_bringup_yaml()
    patch_mission_timeout()
    print("============================================================")
    print("[DONE]")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot_logistics_task")
    print("  source install/setup.bash")
    print("  pkill -f 'mission_btcpp_node|bt_osd_overlay|oak_package_search_node' || true")
    print("  ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task")
    print("============================================================")


if __name__ == "__main__":
    main()
