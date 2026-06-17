#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import re

NODE = Path("/host-home/src/target_capture/oak_guided_pickup/oak_guided_pickup/oak_guided_pickup_node.py")
STAMP = time.strftime("%Y%m%d_%H%M%S")

text = NODE.read_text(encoding="utf-8")
bak = NODE.with_suffix(NODE.suffix + f".bak_feedback_zero_{STAMP}")
bak.write_text(text, encoding="utf-8")
print(f"[backup] {NODE} -> {bak}")

# 原来只发 5 秒，太短；如果从 300mm 以 5mm/s 降到 0，需要约 60 秒。
text = re.sub(
    r"self\.startup_zero_clear_remaining\s*=\s*\d+",
    "self.startup_zero_clear_remaining = 1200",
    text
)

start = text.find("    def publish_startup_zero_clear(self) -> None:\n")
if start < 0:
    raise RuntimeError("Cannot find publish_startup_zero_clear()")

end = text.find("    def now_sec(self) -> float:\n", start)
if end < 0:
    raise RuntimeError("Cannot find def now_sec after publish_startup_zero_clear")

new_func = r'''    def publish_startup_zero_clear(self) -> None:
        """Startup safe return-to-zero using platform feedback.

        Behavior:
        1. Wait for /platform/current_status feedback.
        2. Use actual_lift_z_mm as the starting height.
        3. Publish /platform/target step by step toward 0mm.
        4. Limit command speed by max_lift_speed_mmps.
        5. Stop immediately if a real pickup request becomes active.
        """
        if getattr(self, 'startup_zero_clear_remaining', 0) <= 0:
            timer = getattr(self, 'startup_zero_clear_timer', None)
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
            return

        if getattr(self, 'active', False):
            self.startup_zero_clear_remaining = 0
            timer = getattr(self, 'startup_zero_clear_timer', None)
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
            self.get_logger().info('[startup_feedback_zero] cancelled because pickup request is active')
            return

        now = self.now_sec()

        # Must have real platform feedback. Otherwise we do not know where to start.
        if (not self.platform_status_valid) or (now - self.platform_status_stamp > self.pfloat('platform_current_status_timeout_sec')):
            if now - getattr(self, 'startup_zero_clear_last_log', 0.0) > 1.0:
                self.startup_zero_clear_last_log = now
                self.get_logger().warn('[startup_feedback_zero] waiting for /platform/current_status feedback')
            return

        target = clamp(
            self.pfloat('startup_safe_zero_target_mm') if self.has_parameter('startup_safe_zero_target_mm') else 0.0,
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm')
        )

        # First tick: start from actual feedback height.
        if self.previous_command_lift_z_mm is None:
            self.previous_command_lift_z_mm = float(self.actual_lift_z_mm)

        base = float(self.previous_command_lift_z_mm)

        # Timer is 0.1s, but use configured control_hz to keep consistent with normal lift control.
        max_step = self.pfloat('max_lift_speed_mmps') / max(1.0, self.pfloat('control_hz'))

        delta = target - base
        command = base + clamp(delta, -max_step, max_step)
        command = clamp(command, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm'))

        self.command_lift_z_mm = command
        self.previous_command_lift_z_mm = command

        # Use existing publisher helper so slide/screw are preserved from platform feedback.
        self.publish_platform_target(command, 'startup_feedback_zero')

        self.startup_zero_clear_remaining -= 1

        if now - getattr(self, 'startup_zero_clear_last_log', 0.0) > 0.8:
            self.startup_zero_clear_last_log = now
            self.get_logger().info(
                f'[startup_feedback_zero] actual={self.actual_lift_z_mm:.1f} '
                f'cmd={command:.1f} target={target:.1f} max_step={max_step:.2f} '
                f'remaining={self.startup_zero_clear_remaining}'
            )

        tol = self.pfloat('startup_safe_zero_tolerance_mm') if self.has_parameter('startup_safe_zero_tolerance_mm') else 3.0
        if abs(self.actual_lift_z_mm - target) <= tol and abs(command - target) <= tol:
            self.publish_platform_target(target, 'startup_feedback_zero_done')
            self.startup_zero_clear_remaining = 0
            timer = getattr(self, 'startup_zero_clear_timer', None)
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
            self.get_logger().info('[startup_feedback_zero] done')
            return

'''

text = text[:start] + new_func + text[end:]

NODE.write_text(text, encoding="utf-8")
print("[done] patched startup feedback zero")
