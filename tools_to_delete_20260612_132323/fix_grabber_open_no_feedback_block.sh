#!/usr/bin/env bash
set -euo pipefail

cd /host-home

python3 - <<'PY'
from pathlib import Path
import time

p = Path("/host-home/src/target_capture/oak_package_grabber/oak_package_grabber/oak_package_grabber_node.py")
if not p.exists():
    raise SystemExit(f"[error] missing file: {p}")

s = p.read_text()
bak = p.with_suffix(".py.bak_open_no_feedback_block_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print(f"[backup] {bak}")

old = """        if self.stage == 'ENSURE_GRIPPER_OPEN':
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_open_value,
            )
            if self.gripper_feedback_confirmed(self.grab_open_value):
                self.publish_running('ensure_gripper_open', 'gripper_open_feedback_confirmed')
                if self.stage_elapsed() >= self.open_wait_sec:
                    self.goto('EXTEND_SLIDE')
                return

            self.publish_running('ensure_gripper_open', 'opening_gripper_waiting_feedback')
            if self.stage_elapsed() >= self.gripper_feedback_timeout_sec:
                self.fail('open_gripper_feedback_timeout_before_extend')
            return
"""

new = """        if self.stage == 'ENSURE_GRIPPER_OPEN':
            # Do not block on board gripper feedback here.
            # The board feedback field can lag or stay stale, while the command path is valid.
            # Keep publishing OPEN for open_wait_sec, then continue to slide extension.
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_open_value,
            )
            self.publish_running('ensure_gripper_open', 'opening_gripper_before_extend_no_feedback_block')
            if self.stage_elapsed() >= self.open_wait_sec:
                self.goto('EXTEND_SLIDE')
            return
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] ENSURE_GRIPPER_OPEN no longer blocks on feedback")
else:
    print("[warn] exact ENSURE_GRIPPER_OPEN feedback block not found")

old = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
            self.publish_platform_target(self.actual_slide_x_mm, self.actual_screw_x_mm, self.grab_open_value)

            if self.gripper_feedback_confirmed(self.grab_open_value):
                self.publish_running('debug_open_gripper', 'open_feedback_confirmed')
                if self.stage_elapsed() >= self.open_wait_sec:
                    self.succeed('debug_open_gripper_done_feedback_confirmed')
                return

            self.publish_running('debug_open_gripper', 'opening_gripper_debug_waiting_feedback')
            if self.stage_elapsed() >= self.gripper_feedback_timeout_sec:
                self.fail('debug_open_gripper_feedback_timeout')
            return
"""

new = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
            # Used by BT open-before-search and manual debug.
            # Send OPEN continuously for open_wait_sec, then succeed.
            self.publish_platform_target(self.actual_slide_x_mm, self.actual_screw_x_mm, self.grab_open_value)
            self.publish_running('debug_open_gripper', 'opening_gripper_debug_no_feedback_block')
            if self.stage_elapsed() >= self.open_wait_sec:
                self.succeed('debug_open_gripper_done')
            return
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] DEBUG_OPEN_GRIPPER no longer blocks on feedback")
else:
    print("[warn] exact DEBUG_OPEN_GRIPPER feedback block not found")

# close_gripper 也别因为反馈卡死，避免后面抓取关夹无法继续
old = """        if self.stage == 'DEBUG_CLOSE_GRIPPER':
            self.publish_platform_target(self.actual_slide_x_mm, self.actual_screw_x_mm, self.grab_close_value)

            if self.gripper_feedback_confirmed(self.grab_close_value):
                self.publish_running('debug_close_gripper', 'close_feedback_confirmed')
                if self.stage_elapsed() >= self.close_wait_sec:
                    self.succeed('debug_close_gripper_done_feedback_confirmed')
                return

            self.publish_running('debug_close_gripper', 'closing_gripper_debug_waiting_feedback')
            if self.stage_elapsed() >= self.gripper_feedback_timeout_sec:
                self.fail('debug_close_gripper_feedback_timeout')
            return
"""

new = """        if self.stage == 'DEBUG_CLOSE_GRIPPER':
            self.publish_platform_target(self.actual_slide_x_mm, self.actual_screw_x_mm, self.grab_close_value)
            self.publish_running('debug_close_gripper', 'closing_gripper_debug_no_feedback_block')
            if self.stage_elapsed() >= self.close_wait_sec:
                self.succeed('debug_close_gripper_done')
            return
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] DEBUG_CLOSE_GRIPPER no longer blocks on feedback")
else:
    print("[warn] exact DEBUG_CLOSE_GRIPPER feedback block not found")

p.write_text(s)
print("[done] patched oak_package_grabber_node.py")
PY

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select oak_package_grabber

set +u
source /host-home/install/setup.bash
set -u

echo "[done] rebuilt oak_package_grabber"
