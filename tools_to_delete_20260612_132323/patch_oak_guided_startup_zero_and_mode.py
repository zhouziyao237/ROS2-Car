#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import re

NODE = Path("/host-home/src/target_capture/oak_guided_pickup/oak_guided_pickup/oak_guided_pickup_node.py")
YAML = Path("/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml")

STAMP = time.strftime("%Y%m%d_%H%M%S")

def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_startup_zero_{STAMP}")
    bak.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    print(f"[backup] {path} -> {bak}")

def patch_node():
    text = NODE.read_text(encoding="utf-8")
    backup(NODE)

    if "enable_startup_safe_zero" not in text:
        text = text.replace(
            "        self.declare_parameter('chassis_control_mode_topic', '/chassis_control_mode')\n",
            "        self.declare_parameter('chassis_control_mode_topic', '/chassis_control_mode')\n"
            "\n"
            "        # Startup safety zero: in IDLE and auto mode, lower lift target to 0 slowly.\n"
            "        # This clears stale platform targets without sudden drop.\n"
            "        self.declare_parameter('enable_startup_safe_zero', True)\n"
            "        self.declare_parameter('startup_safe_zero_target_mm', 0.0)\n"
            "        self.declare_parameter('startup_safe_zero_tolerance_mm', 3.0)\n"
            "        self.declare_parameter('startup_safe_zero_timeout_sec', 120.0)\n"
            "        self.declare_parameter('startup_safe_zero_require_auto', True)\n"
            "\n"
            "        # rm_serial fallback mode source. /chassis_control_mode is primary.\n"
            "        # /platform/current_status.control_mode is only used as fallback.\n"
            "        self.declare_parameter('use_platform_control_mode_fallback', True)\n"
            "        self.declare_parameter('platform_auto_control_mode_value', 1)\n"
            "        self.declare_parameter('platform_manual_control_mode_value', 0)\n"
            "        self.declare_parameter('platform_control_mode_as_auto_if_nonzero', True)\n"
        )

    if "self.startup_zero_done" not in text:
        text = text.replace(
            "        self.last_effective_mode = 'unknown'\n",
            "        self.last_effective_mode = 'unknown'\n"
            "\n"
            "        self.startup_zero_done = False\n"
            "        self.startup_zero_start_time = 0.0\n"
            "        self.startup_zero_last_log_time = 0.0\n"
        )

    if "new_request_clear_previous_command_and_mode_edge" not in text:
        text = text.replace(
            "        self.hold_ticks_remaining = 0\n",
            "        self.hold_ticks_remaining = 0\n"
            "\n"
            "        # new_request_clear_previous_command_and_mode_edge\n"
            "        # Clear stale command from previous run. The new request must start from actual height.\n"
            "        self.previous_command_lift_z_mm = None\n"
            "        self.command_lift_z_mm = None\n"
            "        self.manual_synced_lift_z_mm = None\n"
            "        self.last_effective_mode = 'unknown'\n"
            "        self.startup_zero_done = True\n",
            1
        )

    old_effective = """    def effective_chassis_mode(self) -> str:
        now = self.now_sec()
        if self.chassis_mode != 'unknown' and now - self.chassis_mode_stamp <= self.pfloat('chassis_mode_timeout_sec'):
            return self.chassis_mode
        if self.pbool('allow_control_when_chassis_mode_unknown'):
            return 'auto'
        return 'unknown'
"""
    new_effective = """    def effective_chassis_mode(self) -> str:
        now = self.now_sec()

        # Primary mode source: rm_serial_driver /chassis_control_mode.
        if self.chassis_mode != 'unknown' and now - self.chassis_mode_stamp <= self.pfloat('chassis_mode_timeout_sec'):
            return self.chassis_mode

        # Fallback mode source: rm_serial_driver /platform/current_status.control_mode.
        # This is still rm_serial data, not a new control-mode system.
        if self.pbool('use_platform_control_mode_fallback') and self.platform_status_valid:
            auto_val = self.pint('platform_auto_control_mode_value')
            manual_val = self.pint('platform_manual_control_mode_value')
            cmode = int(self.platform_control_mode)

            if cmode == manual_val:
                return 'manual'
            if cmode == auto_val:
                return 'auto'
            if self.pbool('platform_control_mode_as_auto_if_nonzero') and cmode != 0:
                return 'auto'

        if self.pbool('allow_control_when_chassis_mode_unknown'):
            return 'auto'
        return 'unknown'
"""
    if old_effective in text:
        text = text.replace(old_effective, new_effective)
    elif "platform_control_mode_as_auto_if_nonzero" not in text[text.find("def effective_chassis_mode"):text.find("def sync_to_actual_height")]:
        raise RuntimeError("effective_chassis_mode block not matched; stop to avoid unsafe patch")

    if "def handle_startup_safe_zero" not in text:
        insert = """    def handle_startup_safe_zero(self) -> None:
        \"\"\"IDLE safety behavior.

        When the node is idle and rm_serial is auto, slowly clear the platform lift
        target to 0mm. This prevents stale lift target from previous runs, but it
        never publishes a sudden 0mm jump: it uses the same max_lift_speed_mmps
        limiter as visual lift control.
        \"\"\"
        if not self.pbool('enable_startup_safe_zero') or self.startup_zero_done:
            return

        now = self.now_sec()
        if self.startup_zero_start_time <= 0.0:
            self.startup_zero_start_time = now

        if not self.platform_status_valid or now - self.platform_status_stamp > self.pfloat('platform_current_status_timeout_sec'):
            return

        mode = self.effective_chassis_mode()
        if self.pbool('startup_safe_zero_require_auto') and mode != 'auto':
            self.sync_to_actual_height()
            return

        target = clamp(
            self.pfloat('startup_safe_zero_target_mm'),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm')
        )

        if self.previous_command_lift_z_mm is None:
            self.sync_to_actual_height()

        base = float(self.previous_command_lift_z_mm if self.previous_command_lift_z_mm is not None else self.actual_lift_z_mm)
        max_step = self.pfloat('max_lift_speed_mmps') / max(1.0, self.pfloat('control_hz'))
        delta = target - base
        command = base + clamp(delta, -max_step, max_step)
        command = clamp(command, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm'))

        self.command_lift_z_mm = command
        self.previous_command_lift_z_mm = command
        self.manual_synced_lift_z_mm = self.actual_lift_z_mm
        self.publish_platform_target(command, 'startup_safe_zero_limited')

        if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
            self.startup_zero_last_log_time = now
            self.get_logger().info(
                f'[startup_safe_zero] mode={mode} actual={self.actual_lift_z_mm:.1f} '
                f'target={target:.1f} cmd={command:.1f} max_step={max_step:.2f}'
            )

        if abs(self.actual_lift_z_mm - target) <= self.pfloat('startup_safe_zero_tolerance_mm') and abs(command - target) <= self.pfloat('startup_safe_zero_tolerance_mm'):
            self.publish_platform_target(target, 'startup_safe_zero_done')
            self.startup_zero_done = True
            self.get_logger().info('[startup_safe_zero] done')
            return

        if now - self.startup_zero_start_time > self.pfloat('startup_safe_zero_timeout_sec'):
            self.startup_zero_done = True
            self.get_logger().warn('[startup_safe_zero] timeout, stop startup zero loop')
            return

"""
        text = text.replace("    def on_timer(self) -> None:\n", insert + "    def on_timer(self) -> None:\n")

    text = text.replace(
        "        if not self.active:\n            return\n",
        "        if not self.active:\n            self.handle_startup_safe_zero()\n            return\n",
        1
    )

    NODE.write_text(text, encoding="utf-8")
    print(f"[patch] node patched: {NODE}")

def ensure_param_block(yaml_text: str, key: str, line: str, after_key: str) -> str:
    if re.search(rf"^\s*{re.escape(key)}\s*:", yaml_text, re.M):
        return yaml_text
    m = re.search(rf"^(\s*){re.escape(after_key)}\s*:.*$", yaml_text, re.M)
    if not m:
        return yaml_text + "\\n" + line + "\\n"
    insert_pos = yaml_text.find("\\n", m.end()) + 1
    return yaml_text[:insert_pos] + line + "\\n" + yaml_text[insert_pos:]

def patch_yaml():
    text = YAML.read_text(encoding="utf-8")
    backup(YAML)

    # Keep safe slow speed.
    text = re.sub(r"^(\s*)max_lift_speed_mmps\s*:.*$", r"\\1max_lift_speed_mmps: 5.0", text, flags=re.M)

    # Do not invert user's rule: negative is down, positive is up.
    text = re.sub(r"^(\s*)lift_pixel_direction\s*:.*$", r"\\1lift_pixel_direction: 1.0", text, flags=re.M)

    insert_lines = [
        ("enable_startup_safe_zero", "    enable_startup_safe_zero: true", "chassis_mode_timeout_sec"),
        ("startup_safe_zero_target_mm", "    startup_safe_zero_target_mm: 0.0", "enable_startup_safe_zero"),
        ("startup_safe_zero_tolerance_mm", "    startup_safe_zero_tolerance_mm: 3.0", "startup_safe_zero_target_mm"),
        ("startup_safe_zero_timeout_sec", "    startup_safe_zero_timeout_sec: 120.0", "startup_safe_zero_tolerance_mm"),
        ("startup_safe_zero_require_auto", "    startup_safe_zero_require_auto: true", "startup_safe_zero_timeout_sec"),
        ("use_platform_control_mode_fallback", "    use_platform_control_mode_fallback: true", "startup_safe_zero_require_auto"),
        ("platform_auto_control_mode_value", "    platform_auto_control_mode_value: 1", "use_platform_control_mode_fallback"),
        ("platform_manual_control_mode_value", "    platform_manual_control_mode_value: 0", "platform_auto_control_mode_value"),
        ("platform_control_mode_as_auto_if_nonzero", "    platform_control_mode_as_auto_if_nonzero: true", "platform_manual_control_mode_value"),
    ]
    for key, line, after in insert_lines:
        text = ensure_param_block(text, key, line, after)

    YAML.write_text(text, encoding="utf-8")
    print(f"[patch] yaml patched: {YAML}")

def main():
    if not NODE.exists():
        raise FileNotFoundError(NODE)
    if not YAML.exists():
        raise FileNotFoundError(YAML)
    patch_node()
    patch_yaml()
    print("[done] restart oak_guided_pickup_node after patch")

if __name__ == "__main__":
    main()
