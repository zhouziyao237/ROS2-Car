#!/usr/bin/env bash
set -e

echo "[1/9] Stop current robot processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|goal_pose_bridge|rm_serial_driver_node|oak_package_search_node" || true
sleep 3

CPP="/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp"
SRC_BT_DIR="/host-home/src/Navigation/robot_logistics_task/behavior_tree"
NEW_BT="${SRC_BT_DIR}/pickup_task_scan_finally_oak_realign.xml"

echo "[2/9] Backup current corrupted C++..."
STAMP=$(date +%Y%m%d_%H%M%S)
cp "$CPP" "${CPP}.bak_corrupted_${STAMP}"

echo "[3/9] Restore from latest backup before bad patch if available..."
LATEST_BAK=$(ls -1t /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.bak_scan_realign_* 2>/dev/null | head -n 1 || true)

if [ -n "$LATEST_BAK" ]; then
  echo "[RESTORE] $LATEST_BAK -> $CPP"
  cp "$LATEST_BAK" "$CPP"
else
  echo "[WARN] No .bak_scan_realign_* found. Removing illegal injected lines from current file..."
  python3 - <<'PY'
from pathlib import Path
import re

cpp = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
s = cpp.read_text(errors="ignore")

# 删除上一次错误脚本插入的伪代码块
patterns = [
    r'\n\s*// ==== FINAL REALIGN AFTER SCAN ====.*?(?=\n\s*return\s+BT::NodeStatus::FAILURE;)',
    r'\n\s*// Move OAK back to center / shelf front.*',
    r'\n\s*ctx->RCLCPP_INFO\(.*?\);',
    r'\n\s*ctx->RequestOakPackageSearchAlign\(.*?\);.*',
    r'\n\s*ctx->SetOakAlignMaxWZ\(.*?\);.*',
    r'\n\s*ctx->SetOakMoveXYVel\(.*?\);.*',
    r'\n\s*std::this_thread::sleep_for\(.*?\);.*',
]

for p in patterns:
    s = re.sub(p, "", s, flags=re.S)

cpp.write_text(s)
print("[OK] illegal injected pseudo-code removed")
PY
fi

echo "[4/9] Patch C++ safely: add AlwaysFailure node, set default BT, clean scan node..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
NEW_BT = "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml"

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
backup(CPP, "before_safe_finally_realign")

# 1. 确认错误伪代码已经不存在
bad_tokens = [
    "ctx->RequestOakPackageSearchAlign()",
    "ctx->SetOakAlignMaxWZ",
    "ctx->SetOakMoveXYVel",
    "ctx->RCLCPP_INFO",
    "FINAL REALIGN AFTER SCAN",
]

for t in bad_tokens:
    if t in s:
        raise RuntimeError(f"bad injected pseudo-code still exists: {t}")

# 2. 默认 BT XML 指向新的 finally 行为树
s = re.sub(
    r'declare_parameter<std::string>\("bt_xml"\s*,\s*"[^"]+"\s*\);',
    f'declare_parameter<std::string>("bt_xml","{NEW_BT}");',
    s,
    count=1,
)

# 3. 加 AlwaysFailure 节点类，用于扫描失败后：先回正，再让整棵树返回 FAILURE
if "class AlwaysFailure" not in s:
    marker = "class MissionNode:public rclcpp::Node"
    if marker not in s:
        raise RuntimeError("cannot find MissionNode marker to insert AlwaysFailure")

    always_failure_class = r'''
class AlwaysFailure:public BT::SyncActionNode{
public:
    AlwaysFailure(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}
    SIMPLE_PORTS
    BT::NodeStatus tick()override{
        auto ctx=get_ctx(*this);
        ctx->set_state("AlwaysFailure");
        ctx->status("FAILURE","intentional failure after final OAK realign");
        ctx->trace(name(),"FAILURE");
        return BT::NodeStatus::FAILURE;
    }
};

'''
    s = s.replace(marker, always_failure_class + marker)

# 4. 注册 AlwaysFailure
if 'registerNodeType<AlwaysFailure>("AlwaysFailure")' not in s:
    reg_marker = 'factory_.registerNodeType<PublishTaskDone>("PublishTaskDone");'
    if reg_marker not in s:
        raise RuntimeError("cannot find PublishTaskDone registration marker")
    s = s.replace(
        reg_marker,
        'factory_.registerNodeType<AlwaysFailure>("AlwaysFailure");' + reg_marker,
        1
    )

# 5. 删除之前误加的扫描 x/y/wz speed 参数
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

# 6. 扫描角度固定为 40，但不直接回中
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

# 7. 只修改 ScanTargetPackageOnShelf::build_offsets()
cls_pos = s.find("class ScanTargetPackageOnShelf")
if cls_pos < 0:
    raise RuntimeError("cannot find ScanTargetPackageOnShelf")

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
        // 注意：这里不直接加入最后 0° 回中。
        // 扫描成功或失败后，由行为树再次执行 RequestOakPackageSearchAlign，
        // 让 OAK 根据 shelf_front_yaw 闭环重新对正货架正面。
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

# 8. 重新取扫描类，清理扫描阶段 x/y 平移
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

# 压缩重复的 x/y 归零
cls_text = re.sub(
    r'(cmd\.linear\.x=0\.0;cmd\.linear\.y=0\.0;)+ctx->cmd_vel_pub->publish\(cmd\);',
    'cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);',
    cls_text,
)

# 发布速度前保证 x/y 为 0
if "ctx->cmd_vel_pub->publish(cmd);" in cls_text:
    cls_text = cls_text.replace(
        "ctx->cmd_vel_pub->publish(cmd);",
        "cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);"
    )

# 再压缩一次
cls_text = re.sub(
    r'(cmd\.linear\.x=0\.0;cmd\.linear\.y=0\.0;)+ctx->cmd_vel_pub->publish\(cmd\);',
    'cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);',
    cls_text,
)

s = s[:cls_pos] + cls_text + s[cls_close + 1:]

# 9. 最终检查
for t in bad_tokens:
    if t in s:
        raise RuntimeError(f"bad injected pseudo-code reappeared: {t}")

if 'registerNodeType<AlwaysFailure>("AlwaysFailure")' not in s:
    raise RuntimeError("AlwaysFailure not registered")

cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

if "cmd.linear.x=5.6" in cls_text or "cmd.linear.y=5.6" in cls_text:
    raise RuntimeError("scan class still contains x/y=5.6")

for c in [
    "offsets_.push_back(0.0)",
    "offsets_.push_back(-40.0*M_PI/180.0)",
    "offsets_.push_back( 40.0*M_PI/180.0)",
    "cmd.linear.x=0.0",
    "cmd.linear.y=0.0",
]:
    if c not in cls_text:
        raise RuntimeError(f"missing scan code: {c}")

CPP.write_text(s)
print("[OK] C++ safely patched")
PY

echo "[5/9] Write finally-style BT XML..."
mkdir -p "$SRC_BT_DIR"

cat > "$NEW_BT" <<'XML'
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
        扫描成功：Scan -> OAK 重新对正 -> Done
        扫描失败：Fallback 进入第二分支 -> OAK 重新对正 -> AlwaysFailure
        这样无论找到还是没找到，最后都会先回正到货架正面。
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

echo "[OK] wrote $NEW_BT"

echo "[6/9] Patch mission_task.yaml bt_xml..."
python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")
BT = "/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml"

YAMLS = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_bt_xml_finally_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def patch_bt_xml(s: str) -> str:
    if re.search(r'(^\s*bt_xml\s*:\s*).+', s, flags=re.M):
        return re.sub(r'(^\s*bt_xml\s*:\s*).+', r'\g<1>' + BT, s, flags=re.M)

    lines = s.splitlines()
    out = []
    inserted = False
    for line in lines:
        out.append(line)
        if not inserted and re.match(r'^\s*ros__parameters\s*:', line):
            indent = line[:len(line) - len(line.lstrip())] + "  "
            out.append(f"{indent}bt_xml: {BT}")
            inserted = True

    if not inserted:
        out.append(f"bt_xml: {BT}")

    return "\n".join(out) + "\n"

for p in YAMLS:
    if not p.exists():
        print(f"[SKIP] missing {p}")
        continue
    s = p.read_text(errors="ignore")
    old = s
    s = patch_bt_xml(s)
    if s != old:
        backup(p)
        p.write_text(s)
        print(f"[OK] patched {p}")
    else:
        print(f"[NO CHANGE] {p}")
PY

echo "[7/9] Clean rebuild robot_logistics_task and robot2_bringup..."
cd /host-home
rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[8/9] Source install..."
source /host-home/install/setup.bash

echo "[9/9] Check final result..."
echo ""
echo "[CHECK] bad pseudo-code should be gone:"
grep -n "RequestOakPackageSearchAlign()\\|SetOakAlignMaxWZ\\|SetOakMoveXYVel\\|ctx->RCLCPP_INFO\\|FINAL REALIGN AFTER SCAN" "$CPP" || true

echo ""
echo "[CHECK] AlwaysFailure registered:"
grep -n "class AlwaysFailure\\|registerNodeType<AlwaysFailure>" "$CPP" || true

echo ""
echo "[CHECK] BT XML path:"
grep -R -n "bt_xml" /host-home/src/Navigation/robot_logistics_task/config /host-home/install/robot_logistics_task/share/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] finally BT content:"
grep -n "ScanThenFinallyOakRealign\\|ScanSuccess_ReAlign_ThenDone\\|ScanFailure_ReAlign_ThenFail\\|AlwaysFailure\\|RequestOakPackageSearchAlign\\|ScanTargetPackageOnShelf" "$NEW_BT" || true

echo ""
echo "[DONE]"
echo "Restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
