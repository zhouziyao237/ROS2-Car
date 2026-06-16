#!/usr/bin/env bash
set -eo pipefail

YAML="/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAK="${YAML}.bak_bad_yaml_${STAMP}"

cp "$YAML" "$BAK"
echo "[backup] $YAML -> $BAK"

ALIGN_U="$(grep -m1 '^[[:space:]]*align_u_px:' "$BAK" | awk '{print $2}' || true)"
ALIGN_V="$(grep -m1 '^[[:space:]]*align_v_px:' "$BAK" | awk '{print $2}' || true)"
LIFT_DIR="$(grep -m1 '^[[:space:]]*lift_pixel_direction:' "$BAK" | awk '{print $2}' || true)"

[ -n "$ALIGN_U" ] || ALIGN_U="1050.0"
[ -n "$ALIGN_V" ] || ALIGN_V="100.0"
[ -n "$LIFT_DIR" ] || LIFT_DIR="1.0"

cat > "$YAML" <<EOF
oak_guided_pickup:
  ros__parameters:
    request_topic: /target_capture/oak_guided_pickup/request
    status_topic: /target_capture/oak_guided_pickup/status

    datamatrix_topic: /oak/datamatrix_3d_result_json
    rgb_image_topic: /oak/rgb/image_raw
    platform_target_topic: /platform/target
    platform_current_status_topic: /platform/current_status
    chassis_control_mode_topic: /chassis_control_mode
    visual_align_point_topic: /oak/visual_align_point

    require_rm_serial_auto_for_platform_control: true
    allow_control_when_chassis_mode_unknown: false
    allow_control_when_mode_unknown: false
    chassis_mode_timeout_sec: 1.5

    use_platform_control_mode_fallback: true
    platform_auto_control_mode_value: 1
    platform_manual_control_mode_value: 0
    platform_control_mode_as_auto_if_nonzero: true

    enable_startup_safe_zero: true
    startup_safe_zero_target_mm: 0.0
    startup_safe_zero_tolerance_mm: 3.0
    startup_safe_zero_timeout_sec: 120.0
    startup_safe_zero_require_auto: true

    align_u_px: ${ALIGN_U}
    align_v_px: ${ALIGN_V}

    kv_to_lift_mm_per_px: 0.25
    lift_pixel_direction: ${LIFT_DIR}

    v_tolerance_px: 20.0
    stable_required_frames: 5

    max_lift_speed_mmps: 5.0
    control_hz: 10.0
    lift_deadband_mm: 3.0

    platform_min_height_mm: 0.0
    platform_max_height_mm: 330.0

    auto_takeover_hold_ticks: 5
    auto_takeover_first_target_tolerance_mm: 3.0

    target_lost_timeout_sec: 1.0
    stage_timeout_sec: 30.0
    wait_auto_mode_timeout_sec: 10.0
    platform_current_status_timeout_sec: 0.8
    actual_jump_resync_mm: 10.0

    debug_log_period_sec: 0.3
EOF

python3 - <<PY
import yaml
path = "$YAML"
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)
assert "oak_guided_pickup" in data
assert "ros__parameters" in data["oak_guided_pickup"]
print("[OK] yaml format valid:", path)
print("[OK] align_u_px =", data["oak_guided_pickup"]["ros__parameters"]["align_u_px"])
print("[OK] align_v_px =", data["oak_guided_pickup"]["ros__parameters"]["align_v_px"])
print("[OK] max_lift_speed_mmps =", data["oak_guided_pickup"]["ros__parameters"]["max_lift_speed_mmps"])
PY

echo "[done] fixed oak_guided_pickup.yaml"
