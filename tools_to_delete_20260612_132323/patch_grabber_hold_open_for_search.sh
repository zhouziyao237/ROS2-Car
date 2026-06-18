#!/usr/bin/env bash
set -euo pipefail

cd /host-home

echo "========== [1/3] Patch oak_package_grabber_node.py: hold open after open_gripper =========="

python3 - <<'PY'
from pathlib import Path
import time

p = Path("/host-home/src/target_capture/oak_package_grabber/oak_package_grabber/oak_package_grabber_node.py")
if not p.exists():
    raise SystemExit(f"[error] missing file: {p}")

s = p.read_text()
bak = p.with_suffix(".py.bak_hold_open_for_search_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print(f"[backup] {bak}")

# ------------------------------------------------------------------
# 1) Add parameters after log_status_period_sec block
# ------------------------------------------------------------------
old = """        self.log_status_period_sec = float(
            self.declare_parameter('log_status_period_sec', 0.5).value
        )

        # ---------------- Platform feedback cache ----------------
"""

new = """        self.log_status_period_sec = float(
            self.declare_parameter('log_status_period_sec', 0.5).value
        )

        # Search-stage gripper hold:
        # After BT sends open_gripper before OAK search, keep publishing OPEN in background.
        # This prevents the one-shot open command from being too short or overwritten.
        # It will stop when grab_pickup starts, when guided pickup starts, or after max duration.
        self.hold_open_after_open_gripper = bool(
            self.declare_parameter('hold_open_after_open_gripper', True).value
        )
        self.hold_open_max_sec = float(
            self.declare_parameter('hold_open_max_sec', 12.0).value
        )
        self.hold_open_stop_on_guided_pickup_request = bool(
            self.declare_parameter('hold_open_stop_on_guided_pickup_request', True).value
        )
        self.guided_pickup_request_topic = str(
            self.declare_parameter(
                'guided_pickup_request_topic',
                '/target_capture/oak_guided_pickup/request'
            ).value
        )

        # ---------------- Platform feedback cache ----------------
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] added hold-open parameters")
elif "hold_open_after_open_gripper" in s:
    print("[skip] hold-open parameters already exist")
else:
    print("[warn] parameter insertion point not found")


# ------------------------------------------------------------------
# 2) Add state variables after last_cmd_grab_mode block
# ------------------------------------------------------------------
old = """        self.last_cmd_slide_x_mm: float = self.slide_retract_x_mm
        self.last_cmd_screw_x_mm: float = self.screw_retract_x_mm
        self.last_cmd_grab_mode: bool = self.grab_open_value
        self.gripper_feedback_ok_since: Optional[float] = None

        # Debug targets.
"""

new = """        self.last_cmd_slide_x_mm: float = self.slide_retract_x_mm
        self.last_cmd_screw_x_mm: float = self.screw_retract_x_mm
        self.last_cmd_grab_mode: bool = self.grab_open_value
        self.gripper_feedback_ok_since: Optional[float] = None

        # Background hold-open state for search / alignment stage.
        self.hold_open_active: bool = False
        self.hold_open_start_time: float = 0.0
        self.hold_open_request_id: str = ''
        self.hold_open_last_log_time: float = 0.0
        self.request_source: str = ''

        # Debug targets.
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] added hold-open runtime variables")
elif "self.hold_open_active" in s:
    print("[skip] hold-open runtime variables already exist")
else:
    print("[warn] runtime variable insertion point not found")


# ------------------------------------------------------------------
# 3) Add subscription to guided pickup request
# ------------------------------------------------------------------
old = """        self.datamatrix_sub = self.create_subscription(
            String, str(self.datamatrix_topic), self.on_datamatrix, 10
        )
        self.timer = self.create_timer(max(0.001, 1.0 / self.control_hz), self.tick)
"""

new = """        self.datamatrix_sub = self.create_subscription(
            String, str(self.datamatrix_topic), self.on_datamatrix, 10
        )

        # When lift-align starts, stop background hold-open so oak_guided_pickup becomes
        # the only node publishing /platform/target for lift control.
        self.guided_pickup_req_sub = self.create_subscription(
            String, str(self.guided_pickup_request_topic), self.on_guided_pickup_request, 10
        )

        self.timer = self.create_timer(max(0.001, 1.0 / self.control_hz), self.tick)
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] added guided pickup request subscription")
elif "self.guided_pickup_req_sub" in s:
    print("[skip] guided pickup request subscription already exists")
else:
    print("[warn] subscription insertion point not found")


# ------------------------------------------------------------------
# 4) Store source and stop hold on new non-open command
# ------------------------------------------------------------------
old = """        self.command = command
        self.request_id = request_id
        self.pickup_code = normalize_code(req.get('pickup_code', ''))
        self.package_code = normalize_code(req.get('package_code', ''))
"""

new = """        self.command = command
        self.request_id = request_id
        self.request_source = str(req.get('source', ''))
        self.pickup_code = normalize_code(req.get('pickup_code', ''))
        self.package_code = normalize_code(req.get('package_code', ''))

        # Any real motion/grab command should stop the background search-open holder.
        # open_gripper itself may restart the holder when it succeeds.
        if command != 'open_gripper' and self.hold_open_active:
            self.stop_hold_open(f'new_command_{command}')
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] store request source and stop hold on non-open command")
elif "self.request_source = str(req.get('source'" in s:
    print("[skip] request_source already stored")
else:
    print("[warn] request source insertion point not found")


# ------------------------------------------------------------------
# 5) Add hold-open helper functions before publish_status
# ------------------------------------------------------------------
marker = """    def publish_status(self, *, status: str, stage: str, reason: str, request_id: Optional[str] = None,
                       force: bool = False) -> None:
"""

helper = """    def on_guided_pickup_request(self, msg: String) -> None:
        # Lift-align is going to publish /platform/target for lift PID.
        # Stop hold-open to avoid fighting lift commands.
        if self.hold_open_active and self.hold_open_stop_on_guided_pickup_request:
            self.stop_hold_open('guided_pickup_request')

    def start_hold_open(self, reason: str) -> None:
        if not self.hold_open_after_open_gripper:
            return
        self.hold_open_active = True
        self.hold_open_start_time = now_s()
        self.hold_open_request_id = self.request_id
        self.hold_open_last_log_time = 0.0
        self.get_logger().info(
            f\"[hold_open] start request={self.hold_open_request_id} reason={reason} \"
            f\"max_sec={self.hold_open_max_sec:.1f}\"
        )

    def stop_hold_open(self, reason: str) -> None:
        if not self.hold_open_active:
            return
        elapsed = now_s() - self.hold_open_start_time
        self.get_logger().info(
            f\"[hold_open] stop request={self.hold_open_request_id} \"
            f\"reason={reason} elapsed={elapsed:.2f}s\"
        )
        self.hold_open_active = False
        self.hold_open_request_id = ''

    def tick_hold_open(self) -> None:
        if not self.hold_open_active:
            return

        elapsed = now_s() - self.hold_open_start_time
        if elapsed > self.hold_open_max_sec:
            self.stop_hold_open('hold_open_max_sec_reached')
            return

        if not self.platform_status_valid():
            return

        # If chassis/platform is manual, do not spam automatic platform target.
        # It will resume if the chassis goes back to auto within hold_open_max_sec.
        if self.require_auto_mode and self.control_mode != 0:
            return

        # Keep mechanism retracted and gripper OPEN during search.
        # This is only for search/scan/approach preparation.
        # It must stop before lift-align so it does not fight oak_guided_pickup.
        self.publish_platform_target(
            self.slide_retract_x_mm,
            self.screw_retract_x_mm,
            self.grab_open_value,
        )

        t = now_s()
        if self.debug_log and (t - self.hold_open_last_log_time) >= 1.0:
            self.hold_open_last_log_time = t
            self.get_logger().info(
                f\"[hold_open] publishing OPEN for search: \"
                f\"slide={self.slide_retract_x_mm:.1f} screw={self.screw_retract_x_mm:.1f} \"
                f\"grab=0 elapsed={elapsed:.1f}/{self.hold_open_max_sec:.1f}s\"
            )

"""

if helper.strip() in s:
    print("[skip] hold-open helper functions already exist")
elif marker in s:
    s = s.replace(marker, helper + marker, 1)
    print("[patch] added hold-open helper functions")
else:
    print("[warn] publish_status marker not found")


# ------------------------------------------------------------------
# 6) Make DEBUG_OPEN_GRIPPER start hold-open before succeeded
# ------------------------------------------------------------------
old = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
            # Used by:
            # - manual debug command: open_gripper
            # - BT node ExecuteOakPackageOpenBeforeSearch
            #
            # This must behave like the user's manual ros2 topic pub command:
            # publish OPEN for a short duration, then report succeeded.
            # It must NOT fail the BT only because board feedback did not echo 0.
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_open_value,
            )
            self.publish_running(
                'debug_open_gripper',
                'opening_gripper_debug_no_feedback_block'
            )

            if self.stage_elapsed() >= self.open_wait_sec:
                self.succeed('debug_open_gripper_done')
            return
"""

new = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
            # Used by:
            # - manual debug command: open_gripper
            # - BT node ExecuteOakPackageOpenBeforeSearch
            #
            # Publish OPEN for open_wait_sec, report succeeded to BT,
            # and then keep OPEN in the background during search.
            self.publish_platform_target(
                self.slide_retract_x_mm,
                self.screw_retract_x_mm,
                self.grab_open_value,
            )
            self.publish_running(
                'debug_open_gripper',
                'opening_gripper_debug_and_start_hold_open'
            )

            if self.stage_elapsed() >= self.open_wait_sec:
                self.start_hold_open('open_gripper_succeeded')
                self.succeed('debug_open_gripper_done_hold_open_started')
            return
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] DEBUG_OPEN_GRIPPER now starts background hold-open")
elif "debug_open_gripper_done_hold_open_started" in s:
    print("[skip] DEBUG_OPEN_GRIPPER already starts hold-open")
else:
    print("[warn] DEBUG_OPEN_GRIPPER exact block not found")


# ------------------------------------------------------------------
# 7) Tick hold-open while node is idle
# ------------------------------------------------------------------
old = """    def tick(self) -> None:
        if not self.active:
            return

        if self.request_elapsed() > self.timeout_sec:
"""

new = """    def tick(self) -> None:
        if not self.active:
            self.tick_hold_open()
            return

        if self.request_elapsed() > self.timeout_sec:
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] tick() publishes hold-open while idle")
elif "self.tick_hold_open()" in s:
    print("[skip] tick() already calls tick_hold_open")
else:
    print("[warn] tick() insertion point not found")


p.write_text(s)
print("[done] patched oak_package_grabber_node.py")
PY


echo "========== [2/3] Patch YAML parameters =========="

python3 - <<'PY'
from pathlib import Path
import time
import re

files = [
    Path("/host-home/src/target_capture/oak_package_grabber/config/oak_package_grabber.yaml"),
    Path("/host-home/install/oak_package_grabber/share/oak_package_grabber/config/oak_package_grabber.yaml"),
]

for p in files:
    if not p.exists():
        print(f"[skip missing] {p}")
        continue

    s = p.read_text()
    bak = p.with_suffix(".yaml.bak_hold_open_for_search_" + time.strftime("%Y%m%d_%H%M%S"))
    bak.write_text(s)

    ns = s

    def set_or_append(text: str, key: str, value: str) -> str:
        if re.search(rf"(^\\s*){re.escape(key)}\\s*:", text, flags=re.M):
            return re.sub(
                rf"(^\\s*){re.escape(key)}\\s*:.*$",
                rf"\\1{key}: {value}",
                text,
                flags=re.M,
            )
        # Try to append under ros__parameters indentation if available.
        if "ros__parameters:" in text:
            return text.rstrip() + f"\\n    {key}: {value}\\n"
        return text.rstrip() + f"\\n{key}: {value}\\n"

    ns = set_or_append(ns, "hold_open_after_open_gripper", "true")
    ns = set_or_append(ns, "hold_open_max_sec", "12.0")
    ns = set_or_append(ns, "hold_open_stop_on_guided_pickup_request", "true")
    ns = set_or_append(ns, "guided_pickup_request_topic", "/target_capture/oak_guided_pickup/request")
    ns = set_or_append(ns, "open_wait_sec", "1.0")

    p.write_text(ns)
    print(f"[patch yaml] {p}")

print("[done] patched YAML")
PY


echo "========== [3/3] Build oak_package_grabber =========="

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select oak_package_grabber

set +u
source /host-home/install/setup.bash
set -u

echo "========== DONE =========="
echo "[done] oak_package_grabber now keeps gripper OPEN in background after open_gripper"
