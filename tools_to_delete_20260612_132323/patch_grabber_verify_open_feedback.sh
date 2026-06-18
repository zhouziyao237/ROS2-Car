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
bak = p.with_suffix(".py.bak_verify_open_feedback_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print(f"[backup] {bak}")

# 1) 加参数：开/关夹反馈确认时间
old = """        self.open_wait_sec = float(self.declare_parameter('open_wait_sec', 0.4).value)
"""
new = """        self.open_wait_sec = float(self.declare_parameter('open_wait_sec', 0.4).value)
        # Feedback confirmation: do not report open/close success until board feedback agrees.
        self.gripper_feedback_confirm_sec = float(
            self.declare_parameter('gripper_feedback_confirm_sec', 0.20).value
        )
        self.gripper_feedback_timeout_sec = float(
            self.declare_parameter('gripper_feedback_timeout_sec', 5.0).value
        )
"""
if old not in s:
    print("[warn] open_wait_sec parameter block not found, maybe already patched")
else:
    s = s.replace(old, new, 1)
    print("[patch] added gripper feedback parameters")

# 2) 加反馈稳定计时变量
old = """        self.last_cmd_grab_mode: bool = self.grab_open_value

        # Debug targets.
"""
new = """        self.last_cmd_grab_mode: bool = self.grab_open_value
        self.gripper_feedback_ok_since: Optional[float] = None

        # Debug targets.
"""
if old not in s:
    print("[warn] last_cmd_grab_mode block not found, maybe already patched")
else:
    s = s.replace(old, new, 1)
    print("[patch] added gripper_feedback_ok_since")

# 3) goto() 切 stage 时清空反馈稳定计时
old = """    def goto(self, stage: str) -> None:
        self.stage = stage
        self.stage_start_time = now_s()
        self.publish_running(stage.lower(), 'enter_stage', force=True)
"""
new = """    def goto(self, stage: str) -> None:
        self.stage = stage
        self.stage_start_time = now_s()
        self.gripper_feedback_ok_since = None
        self.publish_running(stage.lower(), 'enter_stage', force=True)
"""
if old not in s:
    print("[warn] goto block not found, maybe already patched")
else:
    s = s.replace(old, new, 1)
    print("[patch] reset feedback timer in goto()")

# 4) 在 publish_platform_target 后面插入反馈判断函数
marker = """    def publish_status(self, *, status: str, stage: str, reason: str, request_id: Optional[str] = None,
                       force: bool = False) -> None:
"""
helper = """    def gripper_feedback_matches(self, target_grab_mode: bool) -> bool:
        # Board feedback semantic: current_grab_mode_bool False=open, True=close.
        return bool(self.current_grab_mode_bool) == bool(target_grab_mode)

    def gripper_feedback_confirmed(self, target_grab_mode: bool) -> bool:
        if not self.platform_status_valid():
            self.gripper_feedback_ok_since = None
            return False

        if not self.gripper_feedback_matches(target_grab_mode):
            self.gripper_feedback_ok_since = None
            return False

        t = now_s()
        if self.gripper_feedback_ok_since is None:
            self.gripper_feedback_ok_since = t

        return (t - self.gripper_feedback_ok_since) >= self.gripper_feedback_confirm_sec

"""
if helper.strip() in s:
    print("[skip] gripper feedback helper already exists")
elif marker not in s:
    print("[warn] publish_status marker not found")
else:
    s = s.replace(marker, helper + marker, 1)
    print("[patch] added gripper feedback helper functions")

# 5) 修改 ENSURE_GRIPPER_OPEN：确认反馈 open 后才进入 EXTEND_SLIDE
old = """        if self.stage == 'ENSURE_GRIPPER_OPEN':
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_open_value,
            )
            self.publish_running('ensure_gripper_open', 'opening_gripper')
            if self.stage_elapsed() >= self.open_wait_sec:
                self.goto('EXTEND_SLIDE')
            return
"""
new = """        if self.stage == 'ENSURE_GRIPPER_OPEN':
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
if old not in s:
    print("[warn] ENSURE_GRIPPER_OPEN block not found, maybe already patched")
else:
    s = s.replace(old, new, 1)
    print("[patch] ENSURE_GRIPPER_OPEN now waits feedback before EXTEND_SLIDE")

# 6) 修改 DEBUG_OPEN_GRIPPER：open_gripper 命令必须等反馈为 open 才成功
old = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
            self.publish_platform_target(self.actual_slide_x_mm, self.actual_screw_x_mm, self.grab_open_value)
            self.publish_running('debug_open_gripper', 'opening_gripper_debug')
            if self.stage_elapsed() >= self.open_wait_sec:
                self.succeed('debug_open_gripper_done')
            return
"""
new = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
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
if old not in s:
    print("[warn] DEBUG_OPEN_GRIPPER block not found, maybe already patched")
else:
    s = s.replace(old, new, 1)
    print("[patch] DEBUG_OPEN_GRIPPER now waits feedback")

# 7) 修改 DEBUG_CLOSE_GRIPPER：close 也等反馈
old = """        if self.stage == 'DEBUG_CLOSE_GRIPPER':
            self.publish_platform_target(self.actual_slide_x_mm, self.actual_screw_x_mm, self.grab_close_value)
            self.publish_running('debug_close_gripper', 'closing_gripper_debug')
            if self.stage_elapsed() >= self.close_wait_sec:
                self.succeed('debug_close_gripper_done')
            return
"""
new = """        if self.stage == 'DEBUG_CLOSE_GRIPPER':
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
if old not in s:
    print("[warn] DEBUG_CLOSE_GRIPPER block not found, maybe already patched")
else:
    s = s.replace(old, new, 1)
    print("[patch] DEBUG_CLOSE_GRIPPER now waits feedback")

p.write_text(s)
print("[done] source patched")
PY

# 修改 YAML：把等待时间调长一点，给舵机动作时间
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
        print("[skip missing]", p)
        continue

    s = p.read_text()
    bak = p.with_suffix(".yaml.bak_verify_open_feedback_" + time.strftime("%Y%m%d_%H%M%S"))
    bak.write_text(s)

    ns = s
    ns = re.sub(r"open_wait_sec:\s*[0-9.]+", "open_wait_sec: 1.2", ns)
    ns = re.sub(r"close_wait_sec:\s*[0-9.]+", "close_wait_sec: 1.0", ns)

    if "gripper_feedback_confirm_sec:" not in ns:
        ns = ns.replace("open_wait_sec: 1.2", "open_wait_sec: 1.2\n    gripper_feedback_confirm_sec: 0.20\n    gripper_feedback_timeout_sec: 5.0")

    p.write_text(ns)
    print("[patch yaml]", p)
PY

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select oak_package_grabber

set +u
source /host-home/install/setup.bash
set -u

echo "[done] oak_package_grabber rebuilt with gripper feedback confirmation"
