#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Patch ScanTargetPackageOnShelf: only rotate left/right 60deg, no x/y movement..."

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
        raise RuntimeError(f"missing stable part: {r}")

backup(CPP, "before_fix_scan_only_rotate_60_no_xy")

# ------------------------------------------------------------
# 1. 不再保留扫描阶段 x/y/wz_speed 参数声明和读取
#    Nav2 速度参数不要在 ScanTargetPackageOnShelf 里动。
# ------------------------------------------------------------
for k in [
    "package_scan_x_speed",
    "package_scan_y_speed",
    "package_scan_wz_speed",
]:
    s = re.sub(
        r'\s*declare_parameter<double>\("' + re.escape(k) + r'",[^;]*;',
        '',
        s,
    )
    s = re.sub(
        r'\s*ctx_->' + re.escape(k) + r'\s*=\s*get_parameter\("' + re.escape(k) + r'"\)\.as_double\(\);',
        '',
        s,
    )

# 如果 SharedContext 里被加过这些字段，删掉它们
s = s.replace(",package_scan_x_speed=5.6", "")
s = s.replace(",package_scan_y_speed=5.6", "")
s = s.replace(",package_scan_wz_speed=3.2", "")

# ------------------------------------------------------------
# 2. 默认扫描角度只保留 left/right/step
# ------------------------------------------------------------
s = re.sub(r'package_scan_left_deg\s*=\s*[-0-9.]+', 'package_scan_left_deg=60.0', s)
s = re.sub(r'package_scan_right_deg\s*=\s*[-0-9.]+', 'package_scan_right_deg=60.0', s)
s = re.sub(r'package_scan_step_deg\s*=\s*[-0-9.]+', 'package_scan_step_deg=60.0', s)

s = re.sub(
    r'declare_parameter<double>\("package_scan_left_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_left_deg",60.0);',
    s,
)
s = re.sub(
    r'declare_parameter<double>\("package_scan_right_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_right_deg",60.0);',
    s,
)
s = re.sub(
    r'declare_parameter<double>\("package_scan_step_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_step_deg",60.0);',
    s,
)

# ------------------------------------------------------------
# 3. 只修改 ScanTargetPackageOnShelf 类内部
# ------------------------------------------------------------
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

# 3.1 替换 build_offsets：0 -> -60 -> +60
fn_pos = cls_text.find("build_offsets")
if fn_pos < 0:
    raise RuntimeError("cannot find build_offsets inside ScanTargetPackageOnShelf")

abs_fn_pos = cls_pos + fn_pos
fn_open = s.find("{", abs_fn_pos)
fn_close = find_matching_brace(s, fn_open)

new_build_offsets = r'''{
        offsets_.clear();

        // 只控制扫描角度：中心 -> 左 60 -> 右 60
        // 扫描阶段不控制 x/y 平移速度，x/y 必须为 0。
        ctx->package_scan_left_deg=60.0;
        ctx->package_scan_right_deg=60.0;
        ctx->package_scan_step_deg=60.0;

        offsets_.push_back(0.0);
        offsets_.push_back(-60.0*M_PI/180.0);
        offsets_.push_back( 60.0*M_PI/180.0);

        std::stringstream ss;
        ss<<"[ScanPackage] offsets deg:";
        for(double v:offsets_){
            ss<<" "<<(v*180.0/M_PI);
        }
        RCLCPP_INFO(ctx->node->get_logger(),"%s",ss.str().c_str());
    }'''

s = s[:fn_open] + new_build_offsets + s[fn_close + 1:]

# 重新取 ScanTargetPackageOnShelf 类
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

# 3.2 状态提示改清楚：只扫角度，不扫 x/y
cls_text = re.sub(
    r'ctx->status\("RUNNING",[^;]*?scanning target package Data Matrix on shelf[^;]*?\);',
    (
        'ctx->status("RUNNING",'
        '"asymmetric angle scanning target package Data Matrix on shelf: center=0deg, left=60deg, right=60deg, no linear x/y movement");'
    ),
    cls_text,
    count=1,
    flags=re.S,
)

# 3.3 撤销扫描阶段 x/y=5.6，强制扫描阶段不平移
cls_text = re.sub(r'cmd\.linear\.x\s*=\s*5\.6\s*;', 'cmd.linear.x=0.0;', cls_text)
cls_text = re.sub(r'cmd\.linear\.y\s*=\s*5\.6\s*;', 'cmd.linear.y=0.0;', cls_text)

# 如果有任何扫描前插入的 x/y 赋值，也确保发布前归零
if "ctx->cmd_vel_pub->publish(cmd);" in cls_text:
    cls_text = cls_text.replace(
        "ctx->cmd_vel_pub->publish(cmd);",
        "cmd.linear.x=0.0;cmd.linear.y=0.0;ctx->cmd_vel_pub->publish(cmd);"
    )

# 3.4 撤销强制 3.2 扫描旋转速度，恢复用已有 oak_align 参数
cls_text = re.sub(
    r'cmd\.angular\.z\s*=\s*clamp_abs_min\(ctx->oak_align_kp\*err,\s*3\.2\s*,\s*0\.2\s*\);',
    'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,ctx->oak_align_max_wz,ctx->oak_align_min_wz);',
    cls_text,
)

s = s[:cls_pos] + cls_text + s[cls_close + 1:]

# ------------------------------------------------------------
# 4. 检查：不能再有扫描 x/y=5.6
# ------------------------------------------------------------
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

if "cmd.linear.x=5.6" in cls_text or "cmd.linear.y=5.6" in cls_text:
    raise RuntimeError("scan class still contains cmd.linear.x/y=5.6")

for c in [
    "offsets_.push_back(-60.0*M_PI/180.0)",
    "offsets_.push_back( 60.0*M_PI/180.0)",
    "cmd.linear.x=0.0",
    "cmd.linear.y=0.0",
]:
    if c not in cls_text:
        raise RuntimeError(f"after patch missing in ScanTargetPackageOnShelf: {c}")

CPP.write_text(s)

print("[OK] patched:", CPP)
print("[DONE] scan stage now only rotates:")
print("  offsets: 0 -60 +60")
print("  scan linear.x = 0")
print("  scan linear.y = 0")
print("  scan angular speed uses existing oak_align_max_wz / oak_align_min_wz")
PY

echo "[3/6] Patch YAML: only angle params, remove scan x/y speed params..."

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
    bak = p.with_suffix(p.suffix + f".bak_remove_scan_xy_speed_{STAMP}")
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

        # 只保留角度参数
        replace_vals = {
            "package_scan_range_deg": "60.0",
            "package_scan_left_deg": "60.0",
            "package_scan_right_deg": "60.0",
            "package_scan_step_deg": "60.0",
        }

        for k, v in replace_vals.items():
            pat = r'(^\s*' + re.escape(k) + r'\s*:\s*)[-0-9.]+'
            if re.search(pat, s, flags=re.M):
                s = re.sub(pat, r'\g<1>' + v, s, flags=re.M)

        # 删除扫描 x/y/wz_speed，不让人误会这是扫描速度
        for k in [
            "package_scan_x_speed",
            "package_scan_y_speed",
            "package_scan_wz_speed",
        ]:
            s = re.sub(r'^\s*' + re.escape(k) + r'\s*:\s*[-0-9.]+\s*\n?', '', s, flags=re.M)

        if s != old:
            backup(p)
            p.write_text(s)
            print("[OK] patched:", p)
PY

echo "[4/6] Clean rebuild..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[5/6] Source install..."
source /host-home/install/setup.bash

echo "[6/6] Check result..."

echo ""
echo "[CHECK] Scan class should have 60deg offsets and x/y zero:"
grep -n "offsets deg\|-60.0\|60.0\|cmd.linear.x=0.0\|cmd.linear.y=0.0\|no linear x/y" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] Should NOT have scan x/y=5.6 in C++:"
grep -n "cmd.linear.x=5.6\|cmd.linear.y=5.6\|package_scan_x_speed\|package_scan_y_speed\|package_scan_wz_speed" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] YAML scan angle params:"
grep -R -n "package_scan_range_deg\|package_scan_left_deg\|package_scan_right_deg\|package_scan_step_deg\|package_scan_x_speed\|package_scan_y_speed\|package_scan_wz_speed" \
/host-home/src/Navigation/robot_logistics_task/config \
/host-home/install/robot_logistics_task/share/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[DONE] Now restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
