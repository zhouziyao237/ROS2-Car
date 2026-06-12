#!/usr/bin/env bash
set -e

echo "[1/8] Stop current robot processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|goal_pose_bridge|rm_serial_driver_node|oak_package_search_node" || true
sleep 3

STAMP=$(date +%Y%m%d_%H%M%S)

CPP="/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp"
BT_SRC_DIR="/host-home/src/Navigation/robot_logistics_task/behavior_tree"
BT_SRC="${BT_SRC_DIR}/pickup_task_scan_finally_oak_realign.xml"

echo "[2/8] Clean C++ duplicate AlwaysFailure / bad pseudo code..."
cp "$CPP" "${CPP}.bak_force_finally_${STAMP}"

python3 - <<'PY'
from pathlib import Path
import re

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
s = CPP.read_text(errors="ignore")

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

# 删除自定义 AlwaysFailure 类，保留 XML 里的内置 AlwaysFailure
while True:
    pos = s.find("class AlwaysFailure")
    if pos < 0:
        break

    open_pos = s.find("{", pos)
    close_pos = find_matching_brace(s, open_pos)

    end = close_pos + 1
    while end < len(s) and s[end].isspace():
        end += 1
    if end < len(s) and s[end] == ";":
        end += 1
    while end < len(s) and s[end] in " \t\r\n":
        end += 1

    print(f"[REMOVE] custom AlwaysFailure class chars {pos}-{end}")
    s = s[:pos] + s[end:]

# 删除自定义注册，避免 ID already registered
s = re.sub(
    r'\s*factory_\.registerNodeType<AlwaysFailure>\("AlwaysFailure"\);\s*',
    '',
    s
)

# 删除之前错误脚本插入的伪代码
bad_patterns = [
    r'\n\s*ctx->RCLCPP_INFO\(.*?\);',
    r'\n\s*ctx->RequestOakPackageSearchAlign\(.*?\);.*',
    r'\n\s*ctx->SetOakAlignMaxWZ\(.*?\);.*',
    r'\n\s*ctx->SetOakMoveXYVel\(.*?\);.*',
    r'\n\s*std::this_thread::sleep_for\(.*?\);.*',
    r'\n\s*// ==== FINAL REALIGN AFTER SCAN ====.*?(?=\n)',
]

for p in bad_patterns:
    s = re.sub(p, '', s, flags=re.S)

# 强制默认 bt_xml 指向 finally BT
new_bt = "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml"
s = re.sub(
    r'declare_parameter<std::string>\("bt_xml"\s*,\s*"[^"]+"\s*\);',
    f'declare_parameter<std::string>("bt_xml","{new_bt}");',
    s,
    count=1,
)

# 最终检查
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
        raise RuntimeError(f"bad token still exists: {t}")

CPP.write_text(s)
print("[OK] C++ cleaned and default bt_xml forced")
PY

echo "[3/8] Write finally BT XML..."
mkdir -p "$BT_SRC_DIR"

cat > "$BT_SRC" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="PickupTask_WithScanFinallyOakRealign">
      <ReceiveShelfGoal/>
      <QueryShelfPose/>

      <NavigateToShelf/>
      <WaitNavArrive/>

      <WaitNavSettle/>

      <!-- 第一次 OAK 对正货架正面 -->
      <RequestOakPackageSearchAlign/>
      <WaitAfterOakAlign/>

      <!--
        核心：
        扫描成功：Scan -> OAK重新对正 -> Done
        扫描失败：进入第二分支 -> OAK重新对正 -> AlwaysFailure
      -->
      <Fallback name="ScanThenFinallyOakRealign">
        <Sequence name="ScanSuccess_ReAlign_ThenDone">
          <ScanTargetPackageOnShelf/>
          <RequestOakPackageSearchAlign/>
          <WaitAfterOakAlign/>
          <PublishTaskDone/>
        </Sequence>

        <Sequence name="ScanFailure_ReAlign_ThenFail">
          <RequestOakPackageSearchAlign/>
          <WaitAfterOakAlign/>
          <AlwaysFailure/>
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
XML

echo "[OK] wrote $BT_SRC"

echo "[4/8] Force every config / launch file to use finally BT..."
python3 - <<'PY'
from pathlib import Path
import re
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")
BT = "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml"

ROOTS = [
    Path("/host-home/src/Navigation/robot_logistics_task"),
    Path("/host-home/src/Navigation/robot2_bringup"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_force_finally_bt_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if ".bak" in p.name:
            continue
        if p.suffix not in [".yaml", ".yml", ".py", ".xml"]:
            continue

        s = p.read_text(errors="ignore")
        old = s

        # 1. YAML 里的 bt_xml:
        if re.search(r'(^\s*bt_xml\s*:\s*).+', s, flags=re.M):
            s = re.sub(
                r'(^\s*bt_xml\s*:\s*).+',
                r'\g<1>' + BT,
                s,
                flags=re.M,
            )

        # 2. launch / yaml / xml 里硬编码旧 BT 文件名
        s = re.sub(
            r'/host-home/src/Navigation/robot_logistics_task/behavior_tree/[^"\']*pickup[^"\']*\.xml',
            BT,
            s
        )

        s = s.replace(
            "/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_full_groot1.xml",
            BT
        )

        s = s.replace(
            "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_full_groot1.xml",
            BT
        )

        if s != old:
            backup(p)
            p.write_text(s)
            print(f"[OK] patched {p}")

PY

echo "[5/8] Show all current bt_xml references before build..."
grep -R -n "bt_xml\|pickup_task_.*xml\|scan_finally" \
/host-home/src/Navigation/robot_logistics_task \
/host-home/src/Navigation/robot2_bringup 2>/dev/null | grep -v ".bak" || true

echo "[6/8] Clean rebuild task + bringup..."
cd /host-home
rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[7/8] Source install..."
source /host-home/install/setup.bash

echo "[8/8] Final check..."
echo ""
echo "[CHECK] installed BT XML should exist:"
ls -l /host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml || true

echo ""
echo "[CHECK] C++ bad tokens should output nothing:"
grep -n "registerNodeType<AlwaysFailure>\|class AlwaysFailure\|ctx->RequestOakPackageSearchAlign()\|ctx->SetOakAlignMaxWZ\|ctx->SetOakMoveXYVel\|ctx->RCLCPP_INFO" "$CPP" || true

echo ""
echo "[CHECK] BT contains finally branch:"
grep -n "ScanThenFinallyOakRealign\|ScanSuccess_ReAlign_ThenDone\|ScanFailure_ReAlign_ThenFail\|AlwaysFailure\|RequestOakPackageSearchAlign\|ScanTargetPackageOnShelf" "$BT_SRC" || true

echo ""
echo "[DONE]"
echo "Restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
echo ""
echo "After launch, check actual loaded bt_xml:"
echo "ros2 param get /mission_btcpp_node bt_xml"
echo "ros2 topic echo /task/bt_tree_xml --once | grep -E \"ScanThenFinallyOakRealign|ScanFailure_ReAlign_ThenFail|AlwaysFailure\""
