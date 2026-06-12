#!/usr/bin/env bash
set -e

echo "[1/8] Stop current robot processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|goal_pose_bridge|rm_serial_driver_node|oak_package_search_node" || true
sleep 3

echo "[2/8] Patch ScanTargetPackageOnShelf: scan only 0 -> -40 -> +40, no direct return-center, no linear x/y..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")
CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

def backup(p: Path, tag: str):
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def find_matching_brace(s: str, open_pos: int) -> int:
    depth = 0
    in_string = False
    escape = False
    in_line_comment = False
    in_block_comment = False

    for i in range(open_pos, len(s)):
        c = s[i]
        n = s[i + 1] if i + 1 < len(s) else ""

        if in_line_comment:
            if c == "\n":
                in_line_comment = False
            continue

        if in_block_comment:
            if c == "*" and n == "/":
                in_block_comment = False
            continue

        if in_string:
            if escape:
                escape = False
                continue
            if c == "\\":
                escape = True
                continue
            if c == '"':
                in_string = False
            continue

        if c == "/" and n == "/":
            in_line_comment = True
            continue

        if c == "/" and n == "*":
            in_block_comment = True
            continue

        if c == '"':
            in_string = True
            continue

        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i

    raise RuntimeError("matching brace not found")

if not CPP.exists():
    raise FileNotFoundError(CPP)

s = CPP.read_text(errors="ignore")

for required in [
    "class WaitNavSettle",
    "class RequestOakPackageSearchAlign",
    "class WaitAfterOakAlign",
    "class ScanTargetPackageOnShelf",
    "class PublishTaskDone",
]:
    if required not in s:
        raise RuntimeError(f"missing required class: {required}")

backup(CPP, "before_finally_oak_realign")

# 删除之前误加的扫描 x/y/wz speed 参数，避免扫描阶段平移
for k in [
    "package_scan_x_speed",
    "package_scan_y_speed",
    "package_scan_wz_speed",
]:
    s = re.sub(r'\s*declare_parameter<double>\("' + re.escape(k) + r'",[^;]*;', '', s)
    s = re.sub(r'\s*ctx_->' + re.escape(k) + r'\s*=\s*get_parameter\("' + re.escape(k) + r'"\)\.as_double\(\);', '', s)

s = s.replace(",package_scan_x_speed=5.6", "")
s = s.replace(",package_scan_y_speed=5.6", "")
s = s.replace(",package_scan_wz_speed=3.2", "")

# 扫描角度参数保持 40
s = re.sub(r'package_scan_left_deg\s*=\s*[-+0-9.eE]+', 'package_scan_left_deg=40.0', s)
s = re.sub(r'package_scan_right_deg\s*=\s*[-+0-9.eE]+', 'package_scan_right_deg=40.0', s)
s = re.sub(r'package_scan_step_deg\s*=\s*[-+0-9.eE]+', 'package_scan_step_deg=40.0', s)

s = re.sub(
    r'declare_parameter<double>\("package_scan_left_deg",\s*[-+0-9.eE]+\s*\);',
    'declare_parameter<double>("package_scan_left_deg",40.0);',
    s,
)
s = re.sub(
    r'declare_parameter<double>\("package_scan_right_deg",\s*[-+0-9.eE]+\s*\);',
    'declare_parameter<double>("package_scan_right_deg",40.0);',
    s,
)
s = re.sub(
    r'declare_parameter<double>\("package_scan_step_deg",\s*[-+0-9.eE]+\s*\);',
    'declare_parameter<double>("package_scan_step_deg",40.0);',
    s,
)

# 只改 ScanTargetPackageOnShelf::build_offsets()
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

fn_pos = cls_text.find("build_offsets")
if fn_pos < 0:
    raise RuntimeError("cannot find build_offsets inside ScanTargetPackageOnShelf")

abs_fn_pos = cls_pos + fn_pos
fn_open = s.find("{", abs_fn_pos)
fn_close = find_matching_brace(s, fn_open)

new_build_offsets = r'''{
        offsets_.clear();

        // 扫描只负责左右看：
        // 0° = OAK 已经对正后的货架正面
        // -40° = 向左看
        // +40° = 向右看
        //
        // 注意：这里不再加入最后 0° 直接回中。
        // 回中必须由行为树后面的 RequestOakPackageSearchAlign 完成，
        // 这样才是 OAK 根据 shelf_front_yaw 闭环重新对正货架正面。
        ctx->package_scan_left_deg=40.0;
        ctx->package_scan_right_deg=40.0;
        ctx->package_scan_step_deg=40.0;

        offsets_.push_back(0.0);
        offsets_.push_back(-40.0*M_PI/180.0);
        offsets_.push_back( 40.0*M_PI/180.0);

        std::stringstream ss;
        ss<<"[ScanPackage] offsets deg:";
        for(double v:offsets_){
            ss<<" "<<(v*180.0/M_PI);
        }
        RCLCPP_INFO(ctx->node->get_logger(),"%s",ss.str().c_str());
    }'''

s = s[:fn_open] + new_build_offsets + s[fn_close + 1:]

# 重新取扫描类，确保扫描阶段没有 x/y 平移
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

cls_text = re.sub(
    r'ctx->status\("RUNNING",[^;]*?scanning target package Data Matrix on shelf[^;]*?\);',
    'ctx->status("RUNNING","angle scan only: front 0deg -> left -40deg -> right +40deg; final shelf-front realign is handled by RequestOakPackageSearchAlign");',
    cls_text,
    count=1,
    flags=re.S,
)

cls_text = re.sub(r'cmd\.linear\.x\s*=\s*5\.6\s*;', 'cmd.linear.x=0.0;', cls_text)
cls_text = re.sub(r'cmd\.linear\.y\s*=\s*5\.6\s*;', 'cmd.linear.y=0.0;', cls_text)

# 防止重复插入过多归零，只先清理一种重复情况
cls_text = cls_text.replace(
    "cmd.linear.x=0.0;cmd.linear.y=0.0;cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);",
    "cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);"
)

# 每次扫描发布速度前，强制 x/y 为 0
cls_text = cls_text.replace(
    "ctx->cmd_vel_pub->publish(cmd);",
    "cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);"
)

s = s[:cls_pos] + cls_text + s[cls_close + 1:]

# 最终检查
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

if "cmd.linear.x=5.6" in cls_text or "cmd.linear.y=5.6" in cls_text:
    raise RuntimeError("scan class still contains cmd.linear.x/y=5.6")

for c in [
    "offsets_.push_back(0.0)",
    "offsets_.push_back(-40.0*M_PI/180.0)",
    "offsets_.push_back( 40.0*M_PI/180.0)",
    "cmd.linear.x=0.0",
    "cmd.linear.y=0.0",
]:
    if c not in cls_text:
        raise RuntimeError(f"missing in ScanTargetPackageOnShelf: {c}")

CPP.write_text(s)

print("[OK] patched ScanTargetPackageOnShelf")
print("Expected scan offsets: 0 -40 40")
print("Final return to shelf front will be done by BT RequestOakPackageSearchAlign.")
PY

echo "[3/8] Create finally-style behavior tree: scan success/failure both realign..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

SRC_BT_DIR = Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree")
SRC_BT_DIR.mkdir(parents=True, exist_ok=True)

BT_FILE = SRC_BT_DIR / "pickup_task_scan_finally_oak_realign.xml"

xml = '''<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="PickupTask_WithScanFinallyOakRealign">
      <ReceiveShelfGoal/>
      <QueryShelfPose/>

      <NavigateToShelf/>
      <WaitNavArrive/>

      <WaitNavSettle/>

      <!-- 初次 OAK 对正货架正面 -->
      <RequestOakPackageSearchAlign/>
      <WaitAfterOakAlign/>

      <!--
        关键结构：
        1. 扫描成功：扫描 -> OAK 再次对正 -> 完成任务
        2. 扫描失败：也先 OAK 再次对正 -> 然后返回 FAILURE
        这样无论有没有找到包裹，最后都会先回到货架正面。
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
'''

if BT_FILE.exists():
    bak = BT_FILE.with_suffix(BT_FILE.suffix + f".bak_{STAMP}")
    shutil.copy2(BT_FILE, bak)
    print(f"[BAK] {BT_FILE} -> {bak}")

BT_FILE.write_text(xml)
print(f"[OK] wrote BT XML: {BT_FILE}")

# 同步一份到 install，如果 install 目录已存在
INSTALL_BT_DIR = Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree")
if INSTALL_BT_DIR.exists():
    INSTALL_BT_DIR.mkdir(parents=True, exist_ok=True)
    out = INSTALL_BT_DIR / BT_FILE.name
    out.write_text(xml)
    print(f"[OK] wrote install BT XML: {out}")
PY

echo "[4/8] Point mission_task.yaml bt_xml to new finally-style BT..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

BT_SRC = "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml"
BT_INSTALL = "/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml"

YAMLS = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config/mission_task.yaml"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_bt_finally_oak_realign_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def patch_bt_xml(s: str, bt_path: str) -> str:
    if re.search(r'(^\s*bt_xml\s*:\s*).+', s, flags=re.M):
        return re.sub(
            r'(^\s*bt_xml\s*:\s*).+',
            r'\g<1>' + bt_path,
            s,
            flags=re.M,
        )

    # 如果没有 bt_xml，就插入到 ros__parameters 下
    lines = s.splitlines()
    out = []
    inserted = False
    for line in lines:
        out.append(line)
        if not inserted and re.match(r'^\s*ros__parameters\s*:', line):
            indent = line[:len(line) - len(line.lstrip())] + "  "
            out.append(f"{indent}bt_xml: {bt_path}")
            inserted = True
    if not inserted:
        out.append(f"bt_xml: {bt_path}")
    return "\n".join(out) + "\n"

for p in YAMLS:
    if not p.exists():
        print(f"[SKIP] missing YAML: {p}")
        continue

    s = p.read_text(errors="ignore")
    old = s

    bt = BT_SRC if "/src/" in str(p) else BT_INSTALL
    s = patch_bt_xml(s, bt)

    if s != old:
        backup(p)
        p.write_text(s)
        print(f"[OK] patched bt_xml in {p} -> {bt}")
    else:
        print(f"[NO CHANGE] {p}")
PY

echo "[5/8] Patch scan YAML params: 40deg scan, no scan x/y speed..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation/robot_logistics_task/config"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_scan40_finally_oak_realign_{STAMP}")
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
        if p.suffix not in [".yaml", ".yml"]:
            continue

        s = p.read_text(errors="ignore")
        old = s

        if "package_scan" not in s:
            continue

        vals = {
            "package_scan_range_deg": "40.0",
            "package_scan_left_deg": "40.0",
            "package_scan_right_deg": "40.0",
            "package_scan_step_deg": "40.0",
        }

        for k, v in vals.items():
            s = re.sub(
                r'(^\s*' + re.escape(k) + r'\s*:\s*)[-+0-9.eE]+',
                r'\g<1>' + v,
                s,
                flags=re.M,
            )

        for k in [
            "package_scan_x_speed",
            "package_scan_y_speed",
            "package_scan_wz_speed",
        ]:
            s = re.sub(
                r'^\s*' + re.escape(k) + r'\s*:\s*[-+0-9.eE]+\s*\n?',
                '',
                s,
                flags=re.M,
            )

        if s != old:
            backup(p)
            p.write_text(s)
            print(f"[OK] patched scan YAML: {p}")
PY

echo "[6/8] Optional: keep your current Nav speed YAML at move=4.2, turn=2.6 without touching scan..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")
MOVE = "4.2"
TURN = "2.6"

ROOTS = [
    Path("/host-home/src/Navigation/robot2_bringup"),
    Path("/host-home/src/Navigation/nav_stack_core_v2"),
    Path("/host-home/install/robot2_bringup"),
    Path("/host-home/install/nav_stack_core_v2"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_nav_move42_turn26_finally_{STAMP}")
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

        for key in ["max_vel_x", "max_vel_y", "max_speed_xy"]:
            s = patch_scalar(s, key, MOVE)

        for key in ["max_vel_theta", "max_speed_theta"]:
            s = patch_scalar(s, key, TURN)

        if re.search(r'^\s*min_vel_x\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_x", "-4.2")

        if re.search(r'^\s*min_vel_y\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_y", "-4.2")

        s = patch_array(s, "max_velocity", "[4.2, 4.2, 2.6]")
        s = patch_array(s, "min_velocity", "[-4.2, -4.2, -2.6]")

        # 不要过猛，防止一卡一卡
        s = patch_array(s, "max_accel", "[1.6, 1.6, 2.0]")
        s = patch_array(s, "max_decel", "[-1.6, -1.6, -2.0]")

        if s != old:
            backup(p)
            p.write_text(s)
            print(f"[OK] patched Nav YAML: {p}")
PY

echo "[7/8] Clean rebuild..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[8/8] Source install and check..."
source /host-home/install/setup.bash

echo ""
echo "[CHECK] mission bt_xml:"
grep -R -n "bt_xml" \
/host-home/src/Navigation/robot_logistics_task/config \
/host-home/install/robot_logistics_task/share/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] New BT should contain Fallback and AlwaysFailure:"
grep -R -n "ScanThenFinallyOakRealign\|ScanSuccess_ReAlign_ThenDone\|ScanFailure_ReAlign_ThenFail\|AlwaysFailure\|RequestOakPackageSearchAlign\|ScanTargetPackageOnShelf" \
/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml \
/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml 2>/dev/null || true

echo ""
echo "[CHECK] Scan C++ offsets:"
grep -n "offsets deg\|-40.0\|40.0\|final shelf-front realign\|cmd.linear.x=0.0\|cmd.linear.y=0.0" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[DONE]"
echo "Restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
