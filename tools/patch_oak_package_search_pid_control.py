#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

NODE = Path("/host-home/src/target_detection/oak_package_search/oak_package_search/oak_package_search_node.py")
YAML = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")


def backup(p: Path):
    if p.exists():
        bak = p.with_suffix(p.suffix + f".bak_pid_{STAMP}")
        shutil.copy2(p, bak)
        print(f"[BACKUP] {p} -> {bak}")


def patch_node():
    if not NODE.exists():
        raise SystemExit(f"[ERROR] not found: {NODE}")

    s = NODE.read_text(errors="ignore")
    backup(NODE)

    # ------------------------------------------------------------
    # 1. Add PID / direction parameters
    # ------------------------------------------------------------
    marker = "        self.declare_parameter('max_wz', 0.30)\n"
    insert = """        self.declare_parameter('max_wz', 0.30)

        # Direction and PID control.
        # If yaw_error has the correct sign but chassis rotates the wrong way,
        # set yaw_control_sign to -1.0.
        self.declare_parameter('yaw_control_sign', -1.0)
        self.declare_parameter('ki_wz', 0.0)
        self.declare_parameter('kd_wz', 0.06)
        self.declare_parameter('pid_integral_limit', 0.5)
        self.declare_parameter('yaw_settle_count_required', 4)
        self.declare_parameter('stop_publish_repeat', 3)
"""
    if "yaw_control_sign" not in s:
        if marker not in s:
            raise SystemExit("[ERROR] cannot find max_wz declaration block")
        s = s.replace(marker, insert, 1)
        print("[OK] added PID parameters")
    else:
        print("[SKIP] PID parameters already exist")

    # ------------------------------------------------------------
    # 2. Add PID runtime state
    # ------------------------------------------------------------
    marker = "        self.found_result: Any = None\n"
    insert = """        self.found_result: Any = None

        # Yaw PID state.
        self._yaw_pid_i = 0.0
        self._yaw_pid_prev_error = None
        self._yaw_pid_prev_time = None
        self._yaw_settle_count = 0
"""
    if "_yaw_pid_i" not in s:
        if marker not in s:
            raise SystemExit("[ERROR] cannot find found_result state block")
        s = s.replace(marker, insert, 1)
        print("[OK] added PID runtime state")
    else:
        print("[SKIP] PID runtime state already exists")

    # ------------------------------------------------------------
    # 3. Add reset_yaw_controller method before publish_stop
    # ------------------------------------------------------------
    reset_method = """
    def reset_yaw_controller(self) -> None:
        self._yaw_pid_i = 0.0
        self._yaw_pid_prev_error = None
        self._yaw_pid_prev_time = None
        self._yaw_settle_count = 0

"""
    if "def reset_yaw_controller" not in s:
        s = s.replace("    def publish_stop(self) -> None:\n", reset_method + "    def publish_stop(self) -> None:\n", 1)
        print("[OK] added reset_yaw_controller()")
    else:
        print("[SKIP] reset_yaw_controller() already exists")

    # ------------------------------------------------------------
    # 4. Reset PID on state change
    # ------------------------------------------------------------
    old = """    def set_state(self, new_state: str, reason: str) -> None:
        old = self.state
        self.state = new_state
        self.state_start = self.now_sec()
        self.get_logger().info(f'state: {old} -> {new_state}, {reason}')
        self.publish_status(new_state.lower(), reason)
"""
    new = """    def set_state(self, new_state: str, reason: str) -> None:
        old = self.state
        self.state = new_state
        self.state_start = self.now_sec()
        self.reset_yaw_controller()
        self.publish_stop()
        self.get_logger().info(f'state: {old} -> {new_state}, {reason}')
        self.publish_status(new_state.lower(), reason)
"""
    if old in s:
        s = s.replace(old, new, 1)
        print("[OK] patched set_state() to reset PID and publish stop")
    else:
        print("[SKIP] set_state() already patched or pattern changed")

    # ------------------------------------------------------------
    # 5. Replace publish_stop with repeated stop
    # ------------------------------------------------------------
    old = """    def publish_stop(self) -> None:
        if not bool(self.get_parameter('enable_motion').value):
            return
        self.cmd_pub.publish(Twist())
"""
    new = """    def publish_stop(self) -> None:
        if not bool(self.get_parameter('enable_motion').value):
            return
        repeat = int(self.get_parameter('stop_publish_repeat').value) if self.has_parameter('stop_publish_repeat') else 3
        repeat = max(1, repeat)
        for _ in range(repeat):
            self.cmd_pub.publish(Twist())
"""
    if old in s:
        s = s.replace(old, new, 1)
        print("[OK] patched publish_stop()")
    else:
        print("[SKIP] publish_stop() already patched or pattern changed")

    # ------------------------------------------------------------
    # 6. Replace yaw_control_to_target with PID version
    # ------------------------------------------------------------
    pattern = r"    def yaw_control_to_target\(self, target_yaw: float, tolerance_deg: float\).*?\n    def detection_matches\(self\) -> bool:"
    m = re.search(pattern, s, flags=re.S)
    if not m:
        raise SystemExit("[ERROR] cannot find yaw_control_to_target block")

    new_func = r'''    def yaw_control_to_target(self, target_yaw: float, tolerance_deg: float) -> Tuple[bool, Optional[str], Optional[float], Optional[float]]:
        ok, current_yaw, err_msg = self.get_oak_yaw_from_tf()
        if not ok:
            self.publish_stop()
            return False, err_msg, None, None

        yaw_error = normalize_angle(target_yaw - current_yaw)
        tol = math.radians(float(tolerance_deg))

        # Stop first. This must be before min_wz, otherwise the robot may keep
        # rotating near the target because min_wz forces a non-zero command.
        if abs(yaw_error) <= tol:
            self._yaw_settle_count += 1
            self.publish_stop()

            required = int(self.get_parameter('yaw_settle_count_required').value)
            required = max(1, required)

            if self._yaw_settle_count >= required:
                self.reset_yaw_controller()
                return True, None, current_yaw, yaw_error

            return False, None, current_yaw, yaw_error

        self._yaw_settle_count = 0

        now = self.now_sec()
        if self._yaw_pid_prev_time is None:
            dt = 1.0 / max(1.0, float(self.get_parameter('tick_hz').value))
        else:
            dt = max(1e-3, now - float(self._yaw_pid_prev_time))

        kp = float(self.get_parameter('kp_wz').value)
        ki = float(self.get_parameter('ki_wz').value)
        kd = float(self.get_parameter('kd_wz').value)
        min_wz = abs(float(self.get_parameter('min_wz').value))
        max_wz = abs(float(self.get_parameter('max_wz').value))
        control_sign = float(self.get_parameter('yaw_control_sign').value)

        integral_limit = abs(float(self.get_parameter('pid_integral_limit').value))
        self._yaw_pid_i += yaw_error * dt
        self._yaw_pid_i = clamp(self._yaw_pid_i, -integral_limit, integral_limit)

        if self._yaw_pid_prev_error is None:
            derivative = 0.0
        else:
            derivative = normalize_angle(yaw_error - float(self._yaw_pid_prev_error)) / dt

        raw = kp * yaw_error + ki * self._yaw_pid_i + kd * derivative
        wz = control_sign * raw
        wz = clamp(wz, -max_wz, max_wz)

        # Apply min_wz only after the stop-zone check.
        if abs(wz) < min_wz:
            if abs(wz) > 1e-6:
                wz = math.copysign(min_wz, wz)
            else:
                wz = control_sign * math.copysign(min_wz, yaw_error)

        self._yaw_pid_prev_error = yaw_error
        self._yaw_pid_prev_time = now

        self.publish_wz(wz)
        return False, None, current_yaw, yaw_error

    def detection_matches(self) -> bool:'''
    s = s[:m.start()] + new_func + s[m.end():]
    print("[OK] replaced yaw_control_to_target() with PID version")

    # ------------------------------------------------------------
    # 7. Apply yaw_control_sign to scan sweep direction
    # ------------------------------------------------------------
    old = """            # Constant slow sweep toward scan_end_yaw.
            scan_speed = abs(float(self.get_parameter('scan_speed_wz').value))
            wz = math.copysign(scan_speed, end_error)
            self.publish_wz(wz)
"""
    new = """            # Constant slow sweep toward scan_end_yaw.
            scan_speed = abs(float(self.get_parameter('scan_speed_wz').value))
            control_sign = float(self.get_parameter('yaw_control_sign').value)
            wz = control_sign * math.copysign(scan_speed, end_error)
            self.publish_wz(wz)
"""
    if old in s:
        s = s.replace(old, new, 1)
        print("[OK] applied yaw_control_sign to scan sweep")
    else:
        print("[SKIP] scan sweep block already patched or pattern changed")

    NODE.write_text(s)
    print(f"[DONE] patched node: {NODE}")


def patch_yaml():
    if not YAML.exists():
        print(f"[SKIP] bringup_modes.yaml not found: {YAML}")
        return

    backup(YAML)
    cfg = yaml.safe_load(YAML.read_text())
    modules = cfg.get("modules", {})
    mod = modules.get("oak_package_search")
    if not isinstance(mod, dict):
        print("[SKIP] oak_package_search module not found in YAML")
        return

    params = mod.setdefault("parameters", [])
    param_dict = None
    for item in params:
        if isinstance(item, dict):
            param_dict = item
            break

    if param_dict is None:
        param_dict = {}
        params.append(param_dict)

    param_dict.update({
        # 当前你说底盘转向反，所以默认先反向。若后面发现又反，改成 1.0。
        "yaw_control_sign": -1.0,

        # PID：先用 PD，不上 I，避免积分导致停不住。
        "kp_wz": 0.70,
        "ki_wz": 0.0,
        "kd_wz": 0.06,
        "pid_integral_limit": 0.5,

        # 到位停止策略。
        "align_yaw_tolerance_deg": 5.0,
        "scan_start_yaw_tolerance_deg": 5.0,
        "yaw_settle_count_required": 4,
        "stop_publish_repeat": 3,

        # 降低近目标时抖动。
        "min_wz": 0.035,
        "max_wz": 0.18,
    })

    YAML.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    print(f"[DONE] patched YAML: {YAML}")


def main():
    print("============================================================")
    print("[PATCH] oak_package_search PID + direction sign + stop latch")
    print("============================================================")
    patch_node()
    patch_yaml()
    print("============================================================")
    print("[DONE]")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select oak_package_search robot2_bringup")
    print("  source install/setup.bash")
    print("============================================================")


if __name__ == "__main__":
    main()
