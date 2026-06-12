#!/usr/bin/env bash
set -e

echo "[1/7] Stop current robot processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|goal_pose_bridge|rm_serial_driver_node|oak_package_search_node" || true
sleep 3

echo "[2/7] Patch ScanTargetPackageOnShelf: scan only 0 -> -40 -> +40, no direct return-center..."

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

required = [
    "class WaitNavSettle",
    "class RequestOakPackageSearchAlign",
    "class ScanTargetPackageOnShelf",
]

for r in required:
    if r not in s:
        raise RuntimeError(f"missing required part: {r}")

backup(CPP, "before_scan_then_oak_realign")

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

# 扫描角度参数：左右 40
s = re.sub(r'package_scan_left_deg\s*=\s*[-0-9.]+', 'package_scan_left_deg=40.0', s)
s = re.sub(r'package_scan_right_deg\s*=\s*[-0-9.]+', 'package_scan_right_deg=40.0', s)
s = re.sub(r'package_scan_step_deg\s*=\s*[-0-9.]+', 'package_scan_step_deg=40.0', s)

s = re.sub(
    r'declare_parameter<double>\("package_scan_left_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_left_deg",40.0);',
    s,
)
s = re.sub(
    r'declare_parameter<double>\("package_scan_right_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_right_deg",40.0);',
    s,
)
s = re.sub(
    r'declare_parameter<double>\("package_scan_step_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_step_deg",40.0);',
    s,
)

# 只修改 ScanTargetPackageOnShelf::build_offsets()
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
        // 注意：这里不再加入最后 0° 回中。
        // 扫描完成后由行为树再次执行 RequestOakPackageSearchAlign，
        // 用 OAK 对正逻辑重新闭环对正 shelf_front_yaw。
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

# 重新取扫描类，确保扫描阶段不平移
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

cls_text = re.sub(
    r'ctx->status\("RUNNING",[^;]*?scanning target package Data Matrix on shelf[^;]*?\);',
    'ctx->status("RUNNING","angle scan only: front 0deg -> left -40deg -> right +40deg; after scan use OAK realign to shelf front");',
    cls_text,
    count=1,
    flags=re.S,
)

cls_text = re.sub(r'cmd\.linear\.x\s*=\s*5\.6\s*;', 'cmd.linear.x=0.0;', cls_text)
cls_text = re.sub(r'cmd\.linear\.y\s*=\s*5\.6\s*;', 'cmd.linear.y=0.0;', cls_text)

# 每次发布扫描 cmd_vel 前都归零 x/y，防止残留线速度
cls_text = cls_text.replace(
    "ctx->cmd_vel_pub->publish(cmd);",
    "cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);"
)

s = s[:cls_pos] + cls_text + s[cls_close + 1:]

# 检查扫描类
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
print("Return to shelf front will be done by RequestOakPackageSearchAlign after scan.")
PY

echo "[3/7] Patch behavior tree XML/YAML: add RequestOakPackageSearchAlign after ScanTargetPackageOnShelf..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation/robot_logistics_task"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task"),
]

def backup(p: Path, tag: str):
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

patched = []

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if ".bak" in p.name:
            continue
        if p.suffix not in [".xml", ".yaml", ".yml"]:
            continue

        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue

        if "ScanTargetPackageOnShelf" not in s:
            continue

        old = s

        # 如果已经有 after scan realign 标记，就不重复插入
        if "after_scan_oak_realign" in s:
            print(f"[SKIP] already patched: {p}")
            continue

        # 常见格式：<ScanTargetPackageOnShelf/> 后面直接 PublishTaskDone
        # 插入第二次 RequestOakPackageSearchAlign 和 WaitAfterOakAlign
        insert_xml = (
            '<ScanTargetPackageOnShelf/>\n'
            '      <!-- after_scan_oak_realign: scan finished, use OAK alignment logic again to face shelf front -->\n'
            '      <RequestOakPackageSearchAlign/>\n'
            '      <WaitAfterOakAlign/>'
        )

        s = re.sub(
            r'<ScanTargetPackageOnShelf\s*/>',
            insert_xml,
            s,
            count=1,
        )

        if s != old:
            backup(p, "before_after_scan_oak_realign")
            p.write_text(s)
            patched.append(str(p))
            print(f"[OK] patched BT file: {p}")

print("")
print("[SUMMARY] patched BT files:")
for x in patched:
    print("  " + x)

if not patched:
    print("[WARN] No BT XML/YAML was patched. Need manual check:")
    print("grep -R -n \"ScanTargetPackageOnShelf\" /host-home/src/Navigation/robot_logistics_task /host-home/install/robot_logistics_task/share/robot_logistics_task")
PY

echo "[4/7] Patch task YAML scan params: left/right 40, step 40..."

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
    bak = p.with_suffix(p.suffix + f".bak_scan40_oak_realign_{STAMP}")
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

        # 删除容易误解为扫描平移速度的参数
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

echo "[5/7] Rebuild robot_logistics_task and robot2_bringup..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[6/7] Source install..."
source /host-home/install/setup.bash

echo "[7/7] Check patch result..."

echo ""
echo "[CHECK] Scan offsets should be 0 -40 40, no final direct 0:"
grep -n "offsets deg\|-40.0\|40.0\|after scan use OAK realign\|cmd.linear.x=0.0\|cmd.linear.y=0.0" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] Behavior tree should contain Scan -> RequestOakPackageSearchAlign -> WaitAfterOakAlign:"
grep -R -n "ScanTargetPackageOnShelf\|after_scan_oak_realign\|RequestOakPackageSearchAlign\|WaitAfterOakAlign" \
/host-home/src/Navigation/robot_logistics_task \
/host-home/install/robot_logistics_task/share/robot_logistics_task 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[DONE]"
echo "Restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
