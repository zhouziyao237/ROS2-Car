#!/usr/bin/env bash
set -e

echo "[1/6] Stop current processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|goal_pose_bridge|rm_serial_driver_node|oak_package_search_node" || true
sleep 3

CPP="/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp"
STAMP=$(date +%Y%m%d_%H%M%S)

echo "[2/6] Backup C++..."
cp "$CPP" "${CPP}.bak_fix_duplicate_always_failure_${STAMP}"

echo "[3/6] Remove custom AlwaysFailure class and registration..."
python3 - <<'PY'
from pathlib import Path
import re
import shutil
import time

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
STAMP = time.strftime("%Y%m%d_%H%M%S")

def find_matching_brace(s: str, open_pos: int) -> int:
    depth = 0
    in_str = False
    esc = False
    in_line = False
    in_block = False

    for i in range(open_pos, len(s)):
        c = s[i]
        n = s[i+1] if i+1 < len(s) else ""

        if in_line:
            if c == "\n":
                in_line = False
            continue

        if in_block:
            if c == "*" and n == "/":
                in_block = False
            continue

        if in_str:
            if esc:
                esc = False
                continue
            if c == "\\":
                esc = True
                continue
            if c == '"':
                in_str = False
            continue

        if c == "/" and n == "/":
            in_line = True
            continue

        if c == "/" and n == "*":
            in_block = True
            continue

        if c == '"':
            in_str = True
            continue

        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i

    raise RuntimeError("matching brace not found")

s = CPP.read_text(errors="ignore")
old = s

# 1. 删除自定义 class AlwaysFailure，保留 XML 里的内置 AlwaysFailure
while True:
    pos = s.find("class AlwaysFailure")
    if pos < 0:
        break

    open_pos = s.find("{", pos)
    if open_pos < 0:
        raise RuntimeError("found class AlwaysFailure but cannot find {")
    close_pos = find_matching_brace(s, open_pos)

    # 吃掉类结尾的 ; 和后面的空白
    end = close_pos + 1
    while end < len(s) and s[end].isspace():
        end += 1
    if end < len(s) and s[end] == ";":
        end += 1
    while end < len(s) and s[end] in " \t\r\n":
        end += 1

    print(f"[REMOVE] custom class AlwaysFailure at chars {pos}-{end}")
    s = s[:pos] + s[end:]

# 2. 删除所有自定义注册语句，防止与 BT.CPP 内置 AlwaysFailure 重名
s = re.sub(
    r'\s*factory_\.registerNodeType<AlwaysFailure>\("AlwaysFailure"\);\s*',
    '',
    s
)

# 3. 防止之前错误伪代码残留
bad_patterns = [
    r'\n\s*ctx->RCLCPP_INFO\(.*?\);',
    r'\n\s*ctx->RequestOakPackageSearchAlign\(.*?\);.*',
    r'\n\s*ctx->SetOakAlignMaxWZ\(.*?\);.*',
    r'\n\s*ctx->SetOakMoveXYVel\(.*?\);.*',
    r'\n\s*// ==== FINAL REALIGN AFTER SCAN ====.*?(?=\n)',
]

for p in bad_patterns:
    s = re.sub(p, '', s, flags=re.S)

bad_tokens = [
    'registerNodeType<AlwaysFailure>("AlwaysFailure")',
    'class AlwaysFailure',
    'ctx->RequestOakPackageSearchAlign()',
    'ctx->SetOakAlignMaxWZ',
    'ctx->SetOakMoveXYVel',
    'ctx->RCLCPP_INFO',
]

for t in bad_tokens:
    if t in s:
        raise RuntimeError(f"bad token still exists after cleanup: {t}")

if s != old:
    CPP.write_text(s)
    print("[OK] cleaned mission_btcpp_node.cpp")
else:
    print("[NO CHANGE] nothing to clean")

PY

echo "[4/6] Verify BT XML still uses built-in AlwaysFailure..."
grep -R -n "AlwaysFailure\|ScanThenFinallyOakRealign\|RequestOakPackageSearchAlign\|ScanTargetPackageOnShelf" \
/host-home/src/Navigation/robot_logistics_task/behavior_tree \
/host-home/src/Navigation/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo "[5/6] Clean rebuild..."
cd /host-home
rm -rf build/robot_logistics_task install/robot_logistics_task

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task --symlink-install

echo "[6/6] Source and final check..."
source /host-home/install/setup.bash

echo ""
echo "[CHECK] These should output nothing:"
grep -n "registerNodeType<AlwaysFailure>\|class AlwaysFailure\|ctx->RequestOakPackageSearchAlign()\|ctx->SetOakAlignMaxWZ\|ctx->SetOakMoveXYVel\|ctx->RCLCPP_INFO" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[DONE]"
echo "Restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
