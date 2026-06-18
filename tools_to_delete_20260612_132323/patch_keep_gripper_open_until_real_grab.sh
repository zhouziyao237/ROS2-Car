#!/usr/bin/env bash
set -euo pipefail

cd /host-home

echo "========== [1/4] Fix grabber YAML: true=open, false=close, hold until real grab =========="

python3 - <<'PY'
from pathlib import Path
import re
import time

files = [
    Path("/host-home/src/target_capture/oak_package_grabber/config/oak_package_grabber.yaml"),
    Path("/host-home/install/oak_package_grabber/share/oak_package_grabber/config/oak_package_grabber.yaml"),
]

def set_param(text: str, key: str, value: str) -> str:
    if re.search(rf"(^\s*){re.escape(key)}\s*:", text, flags=re.M):
        return re.sub(
            rf"(^\s*){re.escape(key)}\s*:.*$",
            rf"\1{key}: {value}",
            text,
            flags=re.M,
        )
    if "ros__parameters:" in text:
        return text.rstrip() + f"\n    {key}: {value}\n"
    return text.rstrip() + f"\n{key}: {value}\n"

stamp = time.strftime("%Y%m%d_%H%M%S")

for p in files:
    if not p.exists():
        print("[skip missing]", p)
        continue

    s = p.read_text()
    bak = p.with_suffix(p.suffix + ".bak_keep_open_until_grab_" + stamp)
    bak.write_text(s)

    # 你实测结果：
    # close_gripper 原来会物理开爪，所以真实板端语义是：
    # true  = OPEN
    # false = CLOSE
    s = set_param(s, "grab_open_value", "true")
    s = set_param(s, "grab_close_value", "false")

    # 搜索/靠近/升降期间保持开爪。
    s = set_param(s, "open_wait_sec", "3.0")
    s = set_param(s, "hold_open_after_open_gripper", "true")

    # 0.0 表示不靠时间自动停止，直到 grab_pickup / close_gripper 才停止。
    s = set_param(s, "hold_open_max_sec", "0.0")

    # 不要在 oak_guided_pickup 开始时停止开爪保持。
    # 因为你要求升降期间也必须一直开着。
    s = set_param(s, "hold_open_stop_on_guided_pickup_request", "false")
    s = set_param(s, "guided_pickup_request_topic", "/target_capture/oak_guided_pickup/request")

    p.write_text(s)
    print("[patched]", p)
    print("[backup]", bak)

print("[done] grabber config: true=open false=close, hold open until real grab")
PY


echo "========== [2/4] Patch oak_package_grabber_node.py: do not stop hold-open at lift_align =========="

python3 - <<'PY'
from pathlib import Path
import re
import time

p = Path("/host-home/src/target_capture/oak_package_grabber/oak_package_grabber/oak_package_grabber_node.py")
if not p.exists():
    raise SystemExit(f"[error] missing file: {p}")

s = p.read_text()
bak = p.with_suffix(".py.bak_keep_open_until_grab_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print("[backup]", bak)

# 1) hold_open_max_sec <= 0 means no timeout.
old = "        if elapsed > self.hold_open_max_sec:\n            self.stop_hold_open('hold_open_max_sec_reached')\n            return\n"
new = "        if self.hold_open_max_sec > 0.0 and elapsed > self.hold_open_max_sec:\n            self.stop_hold_open('hold_open_max_sec_reached')\n            return\n"

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] hold_open_max_sec <= 0 now means hold forever")
elif "self.hold_open_max_sec > 0.0 and elapsed > self.hold_open_max_sec" in s:
    print("[skip] no-timeout hold_open already patched")
else:
    print("[warn] hold_open timeout block not found")

# 2) If old patch stops hold on guided pickup request, disable that behavior.
old = """    def on_guided_pickup_request(self, msg: String) -> None:
        # Lift-align is going to publish /platform/target for lift PID.
        # Stop hold-open to avoid fighting lift commands.
        if self.hold_open_active and self.hold_open_stop_on_guided_pickup_request:
            self.stop_hold_open('guided_pickup_request')
"""

new = """    def on_guided_pickup_request(self, msg: String) -> None:
        # Do NOT stop hold-open when lift_align starts.
        # Requirement: gripper must stay OPEN during search, approach, and lift_align.
        # The oak_guided_pickup node is also patched to publish target_grab_mode=OPEN
        # while it owns /platform/target for lift control.
        if self.hold_open_active and self.hold_open_stop_on_guided_pickup_request:
            self.get_logger().info(
                '[hold_open] guided_pickup_request observed, but keep holding OPEN until real grab'
            )
        return
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] guided pickup no longer stops hold_open")
elif "keep holding OPEN until real grab" in s:
    print("[skip] guided pickup stop already disabled")
else:
    print("[warn] on_guided_pickup_request exact block not found")

# 3) Stop hold-open only for real grab/close/retract commands, not for lift_align.
old = """        # Any real motion/grab command should stop the background search-open holder.
        # open_gripper itself may restart the holder when it succeeds.
        if command != 'open_gripper' and self.hold_open_active:
            self.stop_hold_open(f'new_command_{command}')
"""

new = """        # Keep OPEN through search / approach / lift-align.
        # Stop the background holder only when this grabber is about to execute
        # the real grab sequence or an explicit manual close/retract command.
        if command in ('grab_pickup', 'close_gripper', 'retract_all') and self.hold_open_active:
            self.stop_hold_open(f'new_command_{command}')
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[patch] non-open commands no longer blindly stop hold_open")
elif "command in ('grab_pickup', 'close_gripper', 'retract_all')" in s:
    print("[skip] selective hold_open stop already patched")
else:
    print("[warn] non-open command stop block not found")

p.write_text(s)
print("[done] patched oak_package_grabber_node.py")
PY


echo "========== [3/4] Patch oak_guided_pickup: keep gripper OPEN while publishing lift target =========="

python3 - <<'PY'
from pathlib import Path
import re
import time

py = Path("/host-home/src/target_capture/oak_guided_pickup/oak_guided_pickup/oak_guided_pickup_node.py")
if not py.exists():
    raise SystemExit(f"[error] missing file: {py}")

s = py.read_text()
bak = py.with_suffix(".py.bak_keep_gripper_open_until_grab_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print("[backup]", bak)

# Add parameters if not present.
if "keep_gripper_open_during_pickup" not in s:
    marker = "        self.declare_parameter('chassis_control_mode_topic', '/chassis_control_mode')\n"
    insert = """        self.declare_parameter('keep_gripper_open_during_pickup', True)
        self.declare_parameter('grab_open_value', True)
"""
    if marker in s:
        s = s.replace(marker, marker + "\n" + insert, 1)
        print("[patch] added keep_gripper_open_during_pickup parameters")
    else:
        print("[warn] parameter marker not found")
else:
    print("[skip] keep_gripper_open_during_pickup parameter already exists")

# Add last_platform_grab_mode if not present.
if "self.last_platform_grab_mode" not in s:
    marker = "        self.actual_screw_x_mm = 0.0\n"
    insert = "        self.last_platform_grab_mode = bool(self.pbool('grab_open_value'))\n"
    if marker in s:
        s = s.replace(marker, marker + insert, 1)
        print("[patch] added last_platform_grab_mode")
    else:
        print("[warn] runtime variable marker not found")
else:
    print("[skip] last_platform_grab_mode already exists")

# Cache current status target_grab_mode if possible.
if "self.last_platform_grab_mode = bool(msg.target_grab_mode)" not in s:
    marker = "        self.actual_screw_x_mm = float(msg.target_screw_x_mm)\n"
    insert = """        if hasattr(msg, 'target_grab_mode'):
            try:
                self.last_platform_grab_mode = bool(msg.target_grab_mode)
            except Exception:
                pass
"""
    if marker in s:
        s = s.replace(marker, marker + insert, 1)
        print("[patch] cache target_grab_mode from platform status")
    else:
        print("[warn] platform status cache marker not found")
else:
    print("[skip] current_status target_grab_mode cache already exists")

# Insert target_grab_mode assignment before every platform_target_pub.publish(msg)
# inside publish_platform_target.
if "keep_gripper_open_during_pickup" in s and "forced_keep_open_until_real_grab" not in s:
    target = "        self.platform_target_pub.publish(msg)\n"
    inject = """        # forced_keep_open_until_real_grab:
        # While this node controls lift during pickup, keep gripper OPEN.
        # On the real board: true=OPEN, false=CLOSE.
        # Do not let the bool default false close the gripper during lift_align.
        if hasattr(msg, 'target_grab_mode'):
            if self.active and self.pbool('keep_gripper_open_during_pickup'):
                msg.target_grab_mode = bool(self.pbool('grab_open_value'))
            else:
                msg.target_grab_mode = bool(self.last_platform_grab_mode)

"""
    if target in s:
        s = s.replace(target, inject + target, 1)
        print("[patch] platform target now carries OPEN during pickup/lift_align")
    else:
        print("[warn] platform_target_pub.publish(msg) marker not found")
elif "forced_keep_open_until_real_grab" in s:
    print("[skip] publish_platform_target already patched")
else:
    print("[warn] keep_gripper_open parameter missing; publish patch skipped")

py.write_text(s)
print("[done] patched oak_guided_pickup_node.py")
PY


echo "========== [4/4] Patch oak_guided_pickup YAML + build =========="

python3 - <<'PY'
from pathlib import Path
import re
import time

files = [
    Path("/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml"),
    Path("/host-home/install/oak_guided_pickup/share/oak_guided_pickup/config/oak_guided_pickup.yaml"),
]

def set_param(text: str, key: str, value: str) -> str:
    if re.search(rf"(^\s*){re.escape(key)}\s*:", text, flags=re.M):
        return re.sub(
            rf"(^\s*){re.escape(key)}\s*:.*$",
            rf"\1{key}: {value}",
            text,
            flags=re.M,
        )
    if "ros__parameters:" in text:
        return text.rstrip() + f"\n    {key}: {value}\n"
    return text.rstrip() + f"\n{key}: {value}\n"

stamp = time.strftime("%Y%m%d_%H%M%S")

for p in files:
    if not p.exists():
        print("[skip missing]", p)
        continue

    s = p.read_text()
    bak = p.with_suffix(p.suffix + ".bak_keep_open_until_grab_" + stamp)
    bak.write_text(s)

    s = set_param(s, "keep_gripper_open_during_pickup", "true")
    s = set_param(s, "grab_open_value", "true")

    p.write_text(s)
    print("[patched]", p)
    print("[backup]", bak)

print("[done] oak_guided_pickup YAML patched")
PY

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select oak_package_grabber oak_guided_pickup

set +u
source /host-home/install/setup.bash
set -u

echo "========== DONE =========="
echo "[done] gripper is kept OPEN through search, approach, and lift_align; only grab_pickup closes it"
