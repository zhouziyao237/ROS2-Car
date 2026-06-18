#!/usr/bin/env bash
set -euo pipefail

cd /host-home

python3 - <<'PY'
from pathlib import Path
import time

yaml_text = """oak_package_grabber_node:
  ros__parameters:
    request_topic: /target_capture/oak_package_grabber/request
    status_topic: /target_capture/oak_package_grabber/status

    platform_target_topic: /platform/target
    platform_current_status_topic: /platform/current_status
    datamatrix_topic: /oak/datamatrix_3d_result_json

    control_hz: 20.0

    current_status_timeout_sec: 1.0
    wait_auto_mode_timeout_sec: 10.0
    action_timeout_sec: 30.0

    # 第一节：平台手臂，距离由用户固定给定。
    pickup_slide_extend_x_mm: 20.0
    slide_retract_x_mm: 0.0
    slide_min_x_mm: 0.0
    slide_max_x_mm: 150.0

    # 第二节：夹爪丝杆，距离由用户固定给定。
    pickup_screw_extend_x_mm: 20.0
    screw_retract_x_mm: 0.0
    screw_min_x_mm: 0.0
    screw_max_x_mm: 150.0

    # 板端夹爪语义：0=open, 1=close。
    grab_open_value: false
    grab_close_value: true

    # 动作等待时间。
    open_wait_sec: 1.0
    close_wait_sec: 1.0
    slide_extend_wait_sec: 1.5
    screw_extend_wait_sec: 1.5
    screw_pullback_wait_sec: 1.5
    slide_pullback_wait_sec: 1.5
    fail_recover_wait_sec: 2.0

    # open_gripper 搜索阶段保持开爪。
    hold_open_after_open_gripper: true
    hold_open_max_sec: 12.0
    hold_open_stop_on_guided_pickup_request: true
    guided_pickup_request_topic: /target_capture/oak_guided_pickup/request

    # 抓取判断：只在关夹后回拉阶段判断 z 是否变小。
    require_z_before_pullback: true
    z_before_pullback_max_age_sec: 1.0
    grab_verify_min_delta_z_m: 0.06
    grab_verify_min_samples: 4
    grab_verify_max_lost_sec: 0.8
    grab_verify_window_sec: 3.0

    # 目标码匹配。
    allow_match_full_pickup_code: true
    allow_match_package_code: true

    # 是否要求 auto 模式。
    require_auto_mode: true

    # 调试日志。
    debug_log: true
    log_status_period_sec: 0.5
"""

files = [
    Path("/host-home/src/target_capture/oak_package_grabber/config/oak_package_grabber.yaml"),
    Path("/host-home/install/oak_package_grabber/share/oak_package_grabber/config/oak_package_grabber.yaml"),
]

stamp = time.strftime("%Y%m%d_%H%M%S")

for p in files:
    if not p.exists():
        print("[skip missing]", p)
        continue

    old = p.read_text(errors="ignore")
    bak = p.with_suffix(p.suffix + ".bak_fix_yaml_" + stamp)
    bak.write_text(old)

    p.write_text(yaml_text)
    print("[fixed]", p)
    print("[backup]", bak)

print("[done] oak_package_grabber.yaml rewritten")
PY
