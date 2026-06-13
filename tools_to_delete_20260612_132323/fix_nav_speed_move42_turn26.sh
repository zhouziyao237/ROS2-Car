#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|goal_pose_bridge|rm_serial_driver_node" || true
sleep 3

echo "[2/6] Patch Nav2 YAML speed permanently: move=4.2, turn=2.6..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

MOVE = "4.2"
TURN = "2.6"

ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/install/robot2_bringup"),
    Path("/host-home/install/nav_stack_core_v2"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_nav_move42_turn26_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def patch_scalar(s: str, key: str, val: str) -> str:
    return re.sub(
        r'(^\s*' + re.escape(key) + r'\s*:\s*)[-+0-9.eE]+',
        r'\g<1>' + val,
        s,
        flags=re.M,
    )

def patch_array(s: str, key: str, arr: str) -> str:
    return re.sub(
        r'(^\s*' + re.escape(key) + r'\s*:\s*)\[[^\]]+\]',
        r'\g<1>' + arr,
        s,
        flags=re.M,
    )

def ensure_velocity_smoother_params(s: str) -> str:
    """
    如果 velocity_smoother 节点存在，但 max_velocity / min_velocity 不存在，
    就插入到 velocity_smoother.ros__parameters 下面。
    """
    if "velocity_smoother:" not in s:
        return s

    if "max_velocity:" in s and "min_velocity:" in s:
        return s

    lines = s.splitlines()
    out = []
    inserted = False
    in_vs = False
    saw_ros_params = False
    base_indent = ""

    for i, line in enumerate(lines):
        stripped = line.strip()

        if re.match(r'^\s*velocity_smoother\s*:', line):
            in_vs = True
            saw_ros_params = False
            base_indent = line[:len(line) - len(line.lstrip())]
            out.append(line)
            continue

        if in_vs and re.match(r'^\s*ros__parameters\s*:', line):
            saw_ros_params = True
            out.append(line)
            indent = line[:len(line) - len(line.lstrip())] + "  "
            if "max_velocity:" not in s:
                out.append(f"{indent}max_velocity: [4.2, 4.2, 2.6]")
            if "min_velocity:" not in s:
                out.append(f"{indent}min_velocity: [-4.2, -4.2, -2.6]")
            if "max_accel:" not in s:
                out.append(f"{indent}max_accel: [1.6, 1.6, 2.0]")
            if "max_decel:" not in s:
                out.append(f"{indent}max_decel: [-1.6, -1.6, -2.0]")
            inserted = True
            continue

        # 离开 velocity_smoother 块
        if in_vs and stripped and not line.startswith(base_indent + " ") and not line.startswith(base_indent + "\t"):
            in_vs = False

        out.append(line)

    return "\n".join(out) + "\n"

patched_files = []

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if ".bak" in p.name:
            continue
        if p.suffix not in [".yaml", ".yml"]:
            continue

        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue

        old = s

        # 只处理 Nav2 / DWB / velocity smoother 相关 YAML
        if not any(k in s for k in [
            "DWBLocalPlanner",
            "FollowPath",
            "controller_server",
            "velocity_smoother",
            "max_vel_x",
            "max_vel_theta",
            "max_velocity",
        ]):
            continue

        # DWB / FollowPath 移动速度
        for key in [
            "max_vel_x",
            "max_vel_y",
            "max_speed_xy",
        ]:
            s = patch_scalar(s, key, MOVE)

        # DWB / FollowPath 旋转速度
        for key in [
            "max_vel_theta",
            "max_speed_theta",
        ]:
            s = patch_scalar(s, key, TURN)

        # 允许全向底盘正负方向移动
        if re.search(r'^\s*min_vel_x\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_x", "-4.2")

        if re.search(r'^\s*min_vel_y\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_y", "-4.2")

        # velocity_smoother 限速同步
        s = patch_array(s, "max_velocity", "[4.2, 4.2, 2.6]")
        s = patch_array(s, "min_velocity", "[-4.2, -4.2, -2.6]")

        # 加速度不要太暴力，避免一卡一卡
        s = patch_array(s, "max_accel", "[1.6, 1.6, 2.0]")
        s = patch_array(s, "max_decel", "[-1.6, -1.6, -2.0]")

        # 如果 velocity_smoother 块存在但没有参数，补进去
        s = ensure_velocity_smoother_params(s)

        if s != old:
            backup(p)
            p.write_text(s)
            patched_files.append(str(p))
            print(f"[OK] patched: {p}")

print("")
print("[SUMMARY] patched files:")
for f in patched_files:
    print("  " + f)

if not patched_files:
    print("[WARN] No Nav2 YAML files were patched. Need to inspect config paths.")
PY

echo "[3/6] Clean rebuild robot2_bringup..."
cd /host-home

rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot2_bringup --symlink-install

echo "[4/6] Source install..."
source /host-home/install/setup.bash

echo "[5/6] Check installed YAML values..."
grep -R -n "max_vel_x\|max_vel_y\|max_speed_xy\|max_vel_theta\|max_velocity\|min_velocity\|max_accel\|max_decel" \
/host-home/install/robot2_bringup \
/host-home/install/nav_stack_core_v2 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[6/6] Done."
echo "Restart with:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
