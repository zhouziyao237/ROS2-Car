#!/usr/bin/env bash
set -e

echo "[1/7] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/7] Patch scan: only angle scan 0 -> -40 -> +40 -> 0, no x/y movement..."

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

for r in [
    "class WaitNavSettle",
    "class RequestOakPackageSearchAlign",
    "class ScanTargetPackageOnShelf",
]:
    if r not in s:
        raise RuntimeError(f"missing stable part: {r}")

backup(CPP, "before_nav56_scan40_fix")

# 删除之前误加到任务层的扫描 x/y/wz speed 参数
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

# 角度默认值
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

# 只替换 ScanTargetPackageOnShelf 类
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

        // 扫描只控制角度：中心 -> 左 40 -> 右 40 -> 回中
        // 5.6 / 3.2 是 Nav2 移动速度参数，不属于扫描 x/y。
        ctx->package_scan_left_deg=40.0;
        ctx->package_scan_right_deg=40.0;
        ctx->package_scan_step_deg=40.0;

        offsets_.push_back(0.0);
        offsets_.push_back(-40.0*M_PI/180.0);
        offsets_.push_back( 40.0*M_PI/180.0);
        offsets_.push_back(0.0);

        std::stringstream ss;
        ss<<"[ScanPackage] offsets deg:";
        for(double v:offsets_){
            ss<<" "<<(v*180.0/M_PI);
        }
        RCLCPP_INFO(ctx->node->get_logger(),"%s",ss.str().c_str());
    }'''

s = s[:fn_open] + new_build_offsets + s[fn_close + 1:]

# 重新取扫描类，确保发布前 x/y 归零
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

cls_text = re.sub(
    r'ctx->status\("RUNNING",[^;]*?scanning target package Data Matrix on shelf[^;]*?\);',
    (
        'ctx->status("RUNNING",'
        '"angle scanning target package Data Matrix on shelf: center=0deg -> left=-40deg -> right=+40deg -> return center; scan linear x/y = 0");'
    ),
    cls_text,
    count=1,
    flags=re.S,
)

cls_text = re.sub(r'cmd\.linear\.x\s*=\s*5\.6\s*;', 'cmd.linear.x=0.0;', cls_text)
cls_text = re.sub(r'cmd\.linear\.y\s*=\s*5\.6\s*;', 'cmd.linear.y=0.0;', cls_text)

cls_text = cls_text.replace(
    "ctx->cmd_vel_pub->publish(cmd);",
    "cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);"
)

# 扫描旋转速度继续用 oak_align 参数，不强写 3.2
cls_text = re.sub(
    r'cmd\.angular\.z\s*=\s*clamp_abs_min\(ctx->oak_align_kp\*err,\s*3\.2\s*,\s*0\.2\s*\);',
    'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,ctx->oak_align_max_wz,ctx->oak_align_min_wz);',
    cls_text,
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
    "offsets_.push_back(-40.0*M_PI/180.0)",
    "offsets_.push_back( 40.0*M_PI/180.0)",
    "cmd.linear.x=0.0",
    "cmd.linear.y=0.0",
]:
    if c not in cls_text:
        raise RuntimeError(f"missing in ScanTargetPackageOnShelf: {c}")

CPP.write_text(s)
print("[OK] patched scan C++:", CPP)
PY

echo "[3/7] Patch Nav2 speed params: move=5.6, turn=3.2..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation/robot2_bringup"),
    Path("/host-home/src/Navigation"),
]

MOVE = "5.6"
TURN = "3.2"

def backup(p):
    bak = p.with_suffix(p.suffix + f".bak_nav_speed_56_32_{STAMP}")
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

        # 只改明显属于 Nav2 / DWB / smoother 的配置
        nav_hint = any(x in s for x in [
            "DWBLocalPlanner",
            "FollowPath",
            "controller_server",
            "velocity_smoother",
            "max_vel_x",
            "max_vel_theta",
            "max_velocity",
        ])

        if not nav_hint:
            continue

        # DWB / controller velocity limits
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

        # 如果有 min_vel_y，给全向底盘允许负方向
        if re.search(r'^\s*min_vel_y\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_y", "-5.6")

        # 如果你的底盘允许倒车，可以保留这句；不想倒车就改回 0.0
        if re.search(r'^\s*min_vel_x\s*:', s, flags=re.M):
            s = patch_scalar(s, "min_vel_x", "-5.6")

        # velocity_smoother 的三维速度限制
        s = re.sub(
            r'(^\s*max_velocity\s*:\s*)\[[^\]]+\]',
            r'\g<1>[5.6, 5.6, 3.2]',
            s,
            flags=re.M,
        )
        s = re.sub(
            r'(^\s*min_velocity\s*:\s*)\[[^\]]+\]',
            r'\g<1>[-5.6, -5.6, -3.2]',
            s,
            flags=re.M,
        )

        if s != old:
            backup(p)
            p.write_text(s)
            print("[OK] patched Nav2 speed:", p)

PY

echo "[4/7] Patch task YAML angle params only..."

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

def backup(p):
    bak = p.with_suffix(p.suffix + f".bak_scan40_angle_only_{STAMP}")
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
                r'(^\s*' + re.escape(k) + r'\s*:\s*)[-0-9.]+',
                r'\g<1>' + v,
                s,
                flags=re.M,
            )

        for k in [
            "package_scan_x_speed",
            "package_scan_y_speed",
            "package_scan_wz_speed",
        ]:
            s = re.sub(r'^\s*' + re.escape(k) + r'\s*:\s*[-0-9.]+\s*\n?', '', s, flags=re.M)

        if s != old:
            backup(p)
            p.write_text(s)
            print("[OK] patched task scan YAML:", p)
PY

echo "[5/7] Clean rebuild..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[6/7] Source install..."
source /host-home/install/setup.bash

echo "[7/7] Check result..."

echo ""
echo "[CHECK] Scan class: 40deg and x/y zero:"
grep -n "offsets deg\|-40.0\|40.0\|cmd.linear.x=0.0\|cmd.linear.y=0.0\|scan linear x/y = 0" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] No scan x/y=5.6 remains:"
grep -n "cmd.linear.x=5.6\|cmd.linear.y=5.6\|package_scan_x_speed\|package_scan_y_speed\|package_scan_wz_speed" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] Nav2 speed params:"
grep -R -n "max_vel_x\|max_vel_y\|max_speed_xy\|max_vel_theta\|max_velocity\|min_velocity" \
/host-home/src/Navigation/robot2_bringup \
/host-home/install/robot2_bringup 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[DONE] Restart with:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
