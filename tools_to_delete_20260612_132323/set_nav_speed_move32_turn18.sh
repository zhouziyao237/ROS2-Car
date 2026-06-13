#!/usr/bin/env bash
set -e

echo "[1/5] Stop current robot processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|rm_serial_driver_node" || true
sleep 2

echo "[2/5] Patch Nav2 speed: move=3.2, turn=1.8..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation/robot2_bringup"),
    Path("/host-home/install/robot2_bringup"),
]

MOVE = "3.2"
TURN = "1.8"

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_nav_move32_turn18_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def patch_scalar(s, key, val):
    return re.sub(
        r'(^\s*' + re.escape(key) + r'\s*:\s*)[-+0-9.eE]+',
        r'\g<1>' + val,
        s,
        flags=re.M,
    )

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

        s = p.read_text(errors="ignore")
        old = s

        if not any(x in s for x in [
            "DWBLocalPlanner",
            "controller_server",
            "velocity_smoother",
            "max_vel_x",
            "max_vel_theta",
            "max_velocity",
        ]):
            continue

        # Nav2 DWB 速度限制
        for key in [
            "max_vel_x",
            "max_vel_y",
            "max_speed_xy",
        ]:
            s = patch_scalar(s, key, MOVE)

        for key in [
            "max_vel_theta",
            "max_speed_theta",
        ]:
            s = patch_scalar(s, key, TURN)

        # 全向底盘允许正负方向
        if re.search(r'^\s*min_vel_x\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_x", "-3.2")

        if re.search(r'^\s*min_vel_y\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_y", "-3.2")

        # velocity_smoother 限速同步
        s = re.sub(
            r'(^\s*max_velocity\s*:\s*)\[[^\]]+\]',
            r'\g<1>[3.2, 3.2, 1.8]',
            s,
            flags=re.M,
        )

        s = re.sub(
            r'(^\s*min_velocity\s*:\s*)\[[^\]]+\]',
            r'\g<1>[-3.2, -3.2, -1.8]',
            s,
            flags=re.M,
        )

        # 加速度稍微放开一点，但不要太激进，减少一卡一卡
        s = re.sub(
            r'(^\s*max_accel\s*:\s*)\[[^\]]+\]',
            r'\g<1>[1.2, 1.2, 1.5]',
            s,
            flags=re.M,
        )

        s = re.sub(
            r'(^\s*max_decel\s*:\s*)\[[^\]]+\]',
            r'\g<1>[-1.2, -1.2, -1.5]',
            s,
            flags=re.M,
        )

        if s != old:
            backup(p)
            p.write_text(s)
            print("[OK] patched:", p)

PY

echo "[3/5] Rebuild robot2_bringup..."
cd /host-home
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot2_bringup --symlink-install

echo "[4/5] Source install..."
source /host-home/install/setup.bash

echo "[5/5] Done."
echo ""
echo "Restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
echo ""
echo "After restart, check:"
echo "ros2 param get /controller_server FollowPath.max_vel_x"
echo "ros2 param get /controller_server FollowPath.max_vel_y"
echo "ros2 param get /controller_server FollowPath.max_vel_theta"
echo "ros2 param get /velocity_smoother max_velocity"
