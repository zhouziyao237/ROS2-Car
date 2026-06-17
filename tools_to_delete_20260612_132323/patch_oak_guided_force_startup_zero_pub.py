#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time

NODE = Path("/host-home/src/target_capture/oak_guided_pickup/oak_guided_pickup/oak_guided_pickup_node.py")
STAMP = time.strftime("%Y%m%d_%H%M%S")

text = NODE.read_text(encoding="utf-8")
bak = NODE.with_suffix(NODE.suffix + f".bak_force_startup_zero_{STAMP}")
bak.write_text(text, encoding="utf-8")
print(f"[backup] {NODE} -> {bak}")

if "startup_zero_clear_remaining" not in text:
    text = text.replace(
        "        self.last_log_time = 0.0\n",
        "        self.last_log_time = 0.0\n"
        "\n"
        "        # Force-clear stale PC platform command on node startup.\n"
        "        # This does not wait for auto mode or platform_current_status.\n"
        "        # It prevents stale PC LIFT CMD from previous runs.\n"
        "        self.startup_zero_clear_remaining = 50\n"
        "        self.startup_zero_clear_last_log = 0.0\n",
        1
    )

if "def publish_startup_zero_clear" not in text:
    insert = r'''    def publish_startup_zero_clear(self) -> None:
        """Publish zero PlatformTarget immediately after startup.

        This clears stale PC LIFT CMD shown by the MCU. It is intentionally
        independent from chassis mode and platform status. If a real pickup
        request becomes active, the startup clear timer stops to avoid fighting
        the task controller.
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
            self.get_logger().info('[startup_zero_clear] cancelled because pickup request is active')
            return

        msg = PlatformTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.target_lift_z_mm = 0.0
        msg.target_slide_x_mm = 0.0
        msg.target_screw_x_mm = 0.0
        self.platform_target_pub.publish(msg)

        self.startup_zero_clear_remaining -= 1

        now = self.now_sec()
        if now - getattr(self, 'startup_zero_clear_last_log', 0.0) > 0.8:
            self.startup_zero_clear_last_log = now
            self.get_logger().info(
                f'[startup_zero_clear] publish /platform/target zero, remaining={self.startup_zero_clear_remaining}'
            )

'''
    text = text.replace("    def now_sec(self) -> float:\n", insert + "    def now_sec(self) -> float:\n", 1)

if "self.startup_zero_clear_timer" not in text:
    text = text.replace(
        "        self.timer = self.create_timer(1.0 / max(1.0, hz), self.on_timer)\n",
        "        self.timer = self.create_timer(1.0 / max(1.0, hz), self.on_timer)\n"
        "        self.startup_zero_clear_timer = self.create_timer(0.1, self.publish_startup_zero_clear)\n"
        "        self.publish_startup_zero_clear()\n",
        1
    )

NODE.write_text(text, encoding="utf-8")
print("[done] patched force startup zero publisher")
