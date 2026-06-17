#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import re

NODE = Path("/host-home/src/target_capture/oak_guided_pickup/oak_guided_pickup/oak_guided_pickup_node.py")
YAML = Path("/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml")
STAMP = time.strftime("%Y%m%d_%H%M%S")

text = NODE.read_text(encoding="utf-8")
bak = NODE.with_suffix(NODE.suffix + f".bak_follow_manual_{STAMP}")
bak.write_text(text, encoding="utf-8")
print(f"[backup] {NODE} -> {bak}")

# 1) 补参数声明
if "startup_safe_zero_confirm_sec" not in text:
    text = text.replace(
        "        self.declare_parameter('startup_safe_zero_require_auto', True)\n",
        "        self.declare_parameter('startup_safe_zero_require_auto', True)\n"
        "        self.declare_parameter('startup_safe_zero_confirm_sec', 3.0)\n"
        "        self.declare_parameter('startup_done_heartbeat_sec', 0.5)\n"
        "        self.declare_parameter('startup_zero_follow_actual_jump_mm', 3.0)\n",
        1
    )

# 2) 补状态变量
if "startup_zero_stable_since" not in text:
    text = text.replace(
        "        self.startup_zero_done_published = False\n",
        "        self.startup_zero_done_published = False\n"
        "        self.startup_zero_stable_since = 0.0\n"
        "        self.startup_zero_last_done_pub_time = 0.0\n"
        "        self.startup_zero_last_mode = 'unknown'\n",
        1
    )

# 3) 替换 publish_startup_zero_done：done 之后持续 heartbeat，防止 gate 错过
start = text.find("    def publish_startup_zero_done(self, reason: str) -> None:\n")
if start < 0:
    raise RuntimeError("Cannot find publish_startup_zero_done()")

end = text.find("    def handle_startup_safe_zero(self) -> None:\n", start)
if end < 0:
    raise RuntimeError("Cannot find handle_startup_safe_zero() after publish_startup_zero_done()")

new_publish_done = r'''    def publish_startup_zero_done(self, reason: str, force: bool = False) -> None:
        """Publish startup-zero done.

        It is intentionally re-published as a heartbeat after done, because
        bringup gate helper processes may start after the first done message.
        """
        now = self.now_sec()
        hb = self.pfloat('startup_done_heartbeat_sec')
        if self.startup_zero_done_published and not force:
            if now - getattr(self, 'startup_zero_last_done_pub_time', 0.0) < hb:
                return

        msg = String()
        msg.data = json.dumps({
            'status': 'done',
            'stage': 'startup_feedback_zero',
            'reason': reason,
            'actual_lift_z_mm': float(self.actual_lift_z_mm),
            'target_lift_z_mm': float(self.pfloat('startup_safe_zero_target_mm')),
            'stamp': now,
        }, ensure_ascii=False)

        self.startup_zero_done_pub.publish(msg)
        self.startup_zero_done_published = True
        self.startup_zero_last_done_pub_time = now

'''
text = text[:start] + new_publish_done + text[end:]

# 4) 替换 startup zero 主逻辑：
#    - manual 下绝不发 /platform/target
#    - actual 近 0 不立刻 done，必须连续 confirm_sec
#    - 每次 auto 控制都从最新 actual 出发，manual 改过高度后能跟上
start = text.find("    def handle_startup_safe_zero(self) -> None:\n")
if start < 0:
    raise RuntimeError("Cannot find handle_startup_safe_zero()")

end = text.find("    def on_timer(self) -> None:\n", start)
if end < 0:
    raise RuntimeError("Cannot find on_timer() after handle_startup_safe_zero()")

new_handle = r'''    def handle_startup_safe_zero(self) -> None:
        """Startup feedback return-to-zero.

        Correct behavior:
        - Never publish /platform/target before /platform/current_status is received.
        - Never publish /platform/target while chassis/rm_serial mode is manual.
        - If manual changes lift height, the next auto command starts from the newest feedback height.
        - Do not declare startup done immediately when actual is near zero; require continuous stable zero.
        - After done, keep publishing startup_zero_done heartbeat so bringup gates cannot miss it.
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

        if (not self.platform_status_valid) or (now - self.platform_status_stamp > self.pfloat('platform_current_status_timeout_sec')):
            self.startup_zero_command_mm = None
            self.previous_command_lift_z_mm = None
            self.command_lift_z_mm = None
            self.startup_zero_stable_since = 0.0
            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().warn('[startup_feedback_zero] waiting for /platform/current_status feedback')
            return

        mode = self.effective_chassis_mode()
        actual = clamp(
            float(self.actual_lift_z_mm),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm')
        )

        # Manual stage: track feedback only. Do NOT send platform target.
        if self.pbool('startup_safe_zero_require_auto') and mode != 'auto':
            self.startup_zero_command_mm = None
            self.previous_command_lift_z_mm = None
            self.command_lift_z_mm = None
            self.manual_synced_lift_z_mm = actual
            self.startup_zero_stable_since = 0.0
            self.startup_zero_last_mode = mode

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().warn(
                    f'[startup_feedback_zero] waiting for auto mode, mode={mode}, actual={actual:.1f}; no platform target published'
                )
            return

        # Just switched back to auto: throw away old command base.
        if self.startup_zero_last_mode != 'auto':
            self.startup_zero_command_mm = None
            self.previous_command_lift_z_mm = None
            self.command_lift_z_mm = None
            self.startup_zero_stable_since = 0.0
            self.get_logger().info(
                f'[startup_feedback_zero] auto takeover, resync from feedback actual={actual:.1f}'
            )
        self.startup_zero_last_mode = 'auto'

        target = clamp(
            self.pfloat('startup_safe_zero_target_mm'),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm'),
        )
        tol = self.pfloat('startup_safe_zero_tolerance_mm')
        confirm_sec = self.pfloat('startup_safe_zero_confirm_sec')

        # Already near zero: do NOT publish zero command. Confirm stable zero first.
        if abs(actual - target) <= tol:
            self.startup_zero_command_mm = None
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

        # Not near zero: reset stable timer and command from actual feedback every tick.
        # This is the key fix: manual height changes are followed through actual feedback.
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

        self.publish_platform_target(command, 'startup_feedback_zero_follow_actual')

        if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
            self.startup_zero_last_log_time = now
            self.get_logger().info(
                f'[startup_feedback_zero] mode={mode} actual={actual:.1f} '
                f'cmd={command:.1f} target={target:.1f} max_step={max_step:.2f}'
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

# 5) YAML 加参数
if YAML.exists():
    y = YAML.read_text(encoding="utf-8")
    ybak = YAML.with_suffix(YAML.suffix + f".bak_follow_manual_{STAMP}")
    ybak.write_text(y, encoding="utf-8")
    print(f"[backup] {YAML} -> {ybak}")

    def add_after(anchor: str, line: str, content: str) -> str:
        if line.split(":")[0] in content:
            return content
        return content.replace(anchor, anchor + line, 1)

    y = add_after(
        "    startup_safe_zero_require_auto: true\n",
        "    startup_safe_zero_confirm_sec: 3.0\n"
        "    startup_done_heartbeat_sec: 0.5\n"
        "    startup_zero_follow_actual_jump_mm: 3.0\n",
        y
    )

    YAML.write_text(y, encoding="utf-8")
    print("[done] patched oak_guided_pickup.yaml")

print("[OK] startup zero now follows latest feedback and does not directly publish zero")
