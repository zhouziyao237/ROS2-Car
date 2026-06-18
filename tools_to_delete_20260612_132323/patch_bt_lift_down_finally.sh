#!/usr/bin/env bash
set -euo pipefail

cd /host-home

echo "========== [1/3] Patch pickup BT XML: always lift down after pickup/grab failure =========="

python3 - <<'PY'
from pathlib import Path
import re
import time

xml_path = Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml")
if not xml_path.exists():
    raise SystemExit(f"[error] missing XML: {xml_path}")

s = xml_path.read_text()
bak = xml_path.with_suffix(".xml.bak_lift_down_finally_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print("[backup]", bak)

marker = "PickupGrabWithLiftDownFinally"

if marker in s:
    print("[skip] XML already has PickupGrabWithLiftDownFinally")
    xml_path.write_text(s)
    raise SystemExit(0)

finally_block = """      <!--
        Pickup finally block:
        - Success path:
            ExecuteOakGuidedPickup -> ExecuteOakPackageGrab -> ExecuteOakGuidedLiftDown -> PublishTaskDone
        - Failure path:
            If ExecuteOakGuidedPickup or ExecuteOakPackageGrab fails,
            still execute ExecuteOakGuidedLiftDown once, then force FAILURE.
        This guarantees the lift goes down regardless of pickup/grab result.
      -->
      <Fallback name="PickupGrabWithLiftDownFinally">
        <Sequence name="PickupGrabMain">
          <ExecuteOakGuidedPickup/>
          <ExecuteOakPackageGrab/>
        </Sequence>
        <ForceFailure name="LiftDownOnPickupOrGrabFailure">
          <ExecuteOakGuidedLiftDown/>
        </ForceFailure>
      </Fallback>

      <!-- Success path lift-down after real grab succeeded -->
      <ExecuteOakGuidedLiftDown/>"""

# Replace the normal consecutive pickup/grab/liftdown chain.
patterns = [
    r"\s*<ExecuteOakGuidedPickup\s*/>\s*"
    r"(?:<!--.*?-->\s*)?"
    r"<ExecuteOakPackageGrab\s*/>\s*"
    r"(?:<!--.*?-->\s*)?"
    r"<ExecuteOakGuidedLiftDown\s*/>",

    r"\s*<ExecuteOakGuidedPickup\s*/>\s*"
    r"(?:<!--.*?-->\s*)?"
    r"<ExecuteOakPackageGrab\s*/>",

    r"\s*<ExecuteOakGuidedPickup\s*/>\s*"
    r"(?:<!--.*?-->\s*)?"
    r"<ExecuteOakGuidedLiftDown\s*/>",
]

patched = False

for pat in patterns:
    ns, n = re.subn(pat, "\n" + finally_block, s, count=1, flags=re.S)
    if n > 0:
        s = ns
        patched = True
        print("[patch] replaced pickup/grab section with finally block")
        break

if not patched:
    raise SystemExit(
        "[error] cannot find ExecuteOakGuidedPickup / ExecuteOakPackageGrab section in XML. "
        "Run grep command below and send output:\n"
        "grep -n \"ExecuteOakGuidedPickup\\|ExecuteOakPackageGrab\\|ExecuteOakGuidedLiftDown\" "
        "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml -A8 -B8"
    )

xml_path.write_text(s)
print("[done] XML patched")
PY


echo "========== [2/3] Check patched XML =========="

grep -n "PickupGrabWithLiftDownFinally\|PickupGrabMain\|LiftDownOnPickupOrGrabFailure\|ExecuteOakGuidedPickup\|ExecuteOakPackageGrab\|ExecuteOakGuidedLiftDown" \
/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml


echo "========== [3/3] Build robot_logistics_task =========="

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select robot_logistics_task

set +u
source /host-home/install/setup.bash
set -u

echo "========== DONE =========="
echo "[done] BT now executes ExecuteOakGuidedLiftDown even if guided pickup or grab fails"
