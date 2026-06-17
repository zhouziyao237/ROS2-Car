#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time

NODE = Path("/host-home/src/target_capture/oak_guided_pickup/oak_guided_pickup/oak_guided_pickup_node.py")
YAML = Path("/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml")
STAMP = time.strftime("%Y%m%d_%H%M%S")

text = NODE.read_text(encoding="utf-8")
bak = NODE.with_suffix(NODE.suffix + f".bak_manual_sync_auto_hold_{STAMP}")
bak.write_text(text, encoding="utf-8")
print(f"[backup] {NODE} -> {bak}")

# 1. Add parameters
if "startup_auto_hold_sec" not in text:
    text = text.replace(
        "        self.declare_parameter('startup_safe_zero_confirm_sec', 3.0)\n",
        "        self.declare_parameter('startup_safe_zero_confirm_sec', 3.0)\n"
        "        self.declare_parameter('startup_auto_hold_sec', 1.0)\n",
        1
    )

# 2. Add state variables
if "startup_zero_auto_hold_until" not in text:
    text = text.replace(
        "        self.startup_zero_last_mode = 'unknown'\n",
        "        self.startup_zero_last_mode = 'unknown'\n"
        "        self.startup_zero_auto_hold_until = 0.0\n"
        "        self.startup_zero_hold_height_mm = None\n",
        1
    )

# 3. Replace handle_startup_safe_zero()
start = text.find("    def handle_startup_safe_zero(self) -> None:\n")
if start < 0:
    raise RuntimeError("Cannot find handle_startup_safe_zero()")

end = text.find("    def on_timer(self) -> None:\n", start)
if end < 0:
    raise RuntimeError("Cannot find on_timer() after handle_startup_safe_zero()")

new_handle = r'''    def handle_startup_safe_zero(self) -> None:
        """Startup feedback return-to-zero with manual-height synchronization.

        Required behavior:
        - Before /platform/current_status arrives: publish nothing.
        - In manual mode: publish target_lift_z_mm = actual_lift_z_mm.
          This keeps the PC platform command synchronized with the height moved by hand/RC.
        - When switching from manual to auto: hold the current feedback height for a short time.
        - Only after the auto hold period, descend toward 0 at max_lift_speed_mmps.
        - Startup gate opens only after feedback is near zero and stable.
        """
        now = self.now_sec()

        if self.startup_zero_done:
            self.publish_startup_zero_done('heartbeat_after_done', force=True)
            return

        if not self.pbool('enable_startup_safe_zero'):
            self.startup_zero_done = True
            self.publish_startup_zero_done('disabled', force=True)
            self.get_logger().info('[startup_feedback_zero] disabled, publish done gate')
            return

        if self.active:
            return

        if self.startup_zero_start_time <= 0.0:
            self.startup_zero_start_time = now

        # No feedback: do not publish anything.
        if (not self.platform_status_valid) or (now - self.platform_status_stamp > self.pfloat('platform_current_status_timeout_sec')):
            self.startup_zero_command_mm = None
            self.previous_command_lift_z_mm = None
            self.command_lift_z_mm = None
            self.startup_zero_stable_since = 0.0
            self.startup_zero_auto_hold_until = 0.0
            self.startup_zero_hold_height_mm = None

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().warn('[startup_feedback_zero] waiting for /platform/current_status feedback; no platform target published')
            return

        mode = self.effective_chassis_mode()
        actual = clamp(
            float(self.actual_lift_z_mm),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm')
        )
        target = clamp(
            self.pfloat('startup_safe_zero_target_mm'),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm'),
        )
        tol = self.pfloat('startup_safe_zero_tolerance_mm')
        confirm_sec = self.pfloat('startup_safe_zero_confirm_sec')

        # Manual mode: synchronize PC target to current hand-controlled height.
        # This is intentionally a publish, but it publishes the SAME height, not zero.
        if self.pbool('startup_safe_zero_require_auto') and mode != 'auto':
            command = actual

            self.startup_zero_command_mm = command
            self.previous_command_lift_z_mm = command
            self.command_lift_z_mm = command
            self.manual_synced_lift_z_mm = actual
            self.startup_zero_stable_since = 0.0
            self.startup_zero_auto_hold_until = 0.0
            self.startup_zero_hold_height_mm = None
            self.startup_zero_last_mode = mode

            self.publish_platform_target(command, 'startup_manual_sync_same_height')

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().warn(
                    f'[startup_feedback_zero] manual sync, mode={mode}, '
                    f'actual={actual:.1f}, cmd={command:.1f}; publish same height, not zero'
                )
            return

        # Just switched into auto: hold current height first, do not descend immediately.
        if self.startup_zero_last_mode != 'auto':
            hold_sec = self.pfloat('startup_auto_hold_sec')
            self.startup_zero_auto_hold_until = now + hold_sec
            self.startup_zero_hold_height_mm = actual
            self.startup_zero_stable_since = 0.0
            self.startup_zero_last_mode = 'auto'

            self.startup_zero_command_mm = actual
            self.previous_command_lift_z_mm = actual
            self.command_lift_z_mm = actual
            self.manual_synced_lift_z_mm = actual

            self.publish_platform_target(actual, 'startup_auto_takeover_hold_height')

            self.get_logger().info(
                f'[startup_feedback_zero] auto takeover, hold feedback height first: '
                f'actual={actual:.1f}, hold_sec={hold_sec:.1f}'
            )
            return

        self.startup_zero_last_mode = 'auto'

        # During auto takeover hold: keep target equal to current feedback height.
        if now < self.startup_zero_auto_hold_until:
            command = actual

            self.startup_zero_command_mm = command
            self.previous_command_lift_z_mm = command
            self.command_lift_z_mm = command
            self.manual_synced_lift_z_mm = actual
            self.startup_zero_stable_since = 0.0

            self.publish_platform_target(command, 'startup_auto_hold_same_height')

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                left = self.startup_zero_auto_hold_until - now
                self.get_logger().info(
                    f'[startup_feedback_zero] auto hold same height, '
                    f'actual={actual:.1f}, cmd={command:.1f}, hold_left={left:.1f}s'
                )
            return

        # Near zero: confirm stable zero. Do not keep forcing zero.
        if abs(actual - target) <= tol:
            self.startup_zero_command_mm = actual
            self.previous_command_lift_z_mm = actual
            self.command_lift_z_mm = actual
            self.manual_synced_lift_z_mm = actual

            if self.startup_zero_stable_since <= 0.0:
                self.startup_zero_stable_since = now

            stable_elapsed = now - self.startup_zero_stable_since

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().info(
                    f'[startup_feedback_zero] zero confirming actual={actual:.1f} '
                    f'target={target:.1f} stable={stable_elapsed:.1f}/{confirm_sec:.1f}; no platform target published'
                )

            if stable_elapsed >= confirm_sec:
                self.startup_zero_done = True
                self.publish_startup_zero_done('reached_zero_by_feedback_stable', force=True)
                self.get_logger().info(
                    f'[startup_feedback_zero] done reason=reached_zero_by_feedback_stable actual={actual:.1f}'
                )
            return

        # Descend toward zero after auto hold.
        self.startup_zero_stable_since = 0.0
        max_step = self.pfloat('max_lift_speed_mmps') / max(1.0, self.pfloat('control_hz'))

        if target <= actual:
            command = max(target, actual - max_step)
        else:
            command = min(target, actual + max_step)

        command = clamp(command, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm'))

        self.startup_zero_command_mm = command
        self.previous_command_lift_z_mm = command
        self.command_lift_z_mm = command
        self.manual_synced_lift_z_mm = actual

        self.publish_platform_target(command, 'startup_feedback_zero_after_auto_hold')

        if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
            self.startup_zero_last_log_time = now
            self.get_logger().info(
                f'[startup_feedback_zero] descending after hold, mode={mode}, '
                f'actual={actual:.1f}, cmd={command:.1f}, target={target:.1f}, max_step={max_step:.2f}'
            )

        if now - self.startup_zero_start_time > self.pfloat('startup_safe_zero_timeout_sec'):
            self.get_logger().error(
                '[startup_feedback_zero] timeout; NOT opening startup gate because lift is not confirmed zero'
            )
            return

'''

text = text[:start] + new_handle + text[end:]
NODE.write_text(text, encoding="utf-8")
print("[done] patched oak_guided_pickup_node.py")

# 4. YAML add parameter
if YAML.exists():
    y = YAML.read_text(encoding="utf-8")
    ybak = YAML.with_suffix(YAML.suffix + f".bak_manual_sync_auto_hold_{STAMP}")
    ybak.write_text(y, encoding="utf-8")
    print(f"[backup] {YAML} -> {ybak}")

    if "startup_auto_hold_sec:" not in y:
        y = y.replace(
            "    startup_safe_zero_confirm_sec: 3.0\n",
            "    startup_safe_zero_confirm_sec: 3.0\n"
            "    startup_auto_hold_sec: 1.0\n",
            1
        )

    YAML.write_text(y, encoding="utf-8")
    print("[done] patched oak_guided_pickup.yaml")

print("[OK] manual now publishes same-height sync; auto holds before descending")
