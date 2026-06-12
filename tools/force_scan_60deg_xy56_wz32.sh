#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Patch ScanTargetPackageOnShelf: left/right 60deg, xy=5.6, wz=3.2..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

LEFT_DEG = 60.0
RIGHT_DEG = 60.0

# 只扫 0, -60, +60，动作最明显
STEP_DEG = 60.0

# 用户指定
SCAN_X_SPEED = 5.6
SCAN_Y_SPEED = 5.6
SCAN_WZ_SPEED = 3.2

SCAN_WAIT_SEC = 0.05
SCAN_TIMEOUT_SEC = 60.0


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
old = s

required = [
    "class WaitNavSettle",
    "class RequestOakPackageSearchAlign",
    "class ScanTargetPackageOnShelf",
]

for r in required:
    if r not in s:
        raise RuntimeError(f"missing stable part: {r}")

backup(CPP, "before_force_scan_60deg_xy56_wz32")

# ------------------------------------------------------------
# 1. SharedContext 加扫描速度参数
# ------------------------------------------------------------
if "package_scan_x_speed" not in s:
    s = re.sub(
        r'(package_scan_wz_degps\s*=\s*[-0-9.]+)',
        r'\1,package_scan_x_speed=5.6,package_scan_y_speed=5.6,package_scan_wz_speed=3.2',
        s,
        count=1,
    )

# ------------------------------------------------------------
# 2. 默认参数强制写成目标值
# ------------------------------------------------------------
s = re.sub(r'package_scan_left_deg\s*=\s*[-0-9.]+', 'package_scan_left_deg=60.0', s)
s = re.sub(r'package_scan_right_deg\s*=\s*[-0-9.]+', 'package_scan_right_deg=60.0', s)
s = re.sub(r'package_scan_step_deg\s*=\s*[-0-9.]+', 'package_scan_step_deg=60.0', s)
s = re.sub(r'package_scan_wait_sec\s*=\s*[-0-9.]+', 'package_scan_wait_sec=0.05', s)
s = re.sub(r'package_scan_timeout_sec\s*=\s*[-0-9.]+', 'package_scan_timeout_sec=60.0', s)
s = re.sub(r'package_scan_wz_degps\s*=\s*[-0-9.]+', 'package_scan_wz_degps=3.2', s)

# ------------------------------------------------------------
# 3. declare_parameter 强制写入
# ------------------------------------------------------------
declare_block = (
    'declare_parameter<double>("package_scan_range_deg",60.0);'
    'declare_parameter<double>("package_scan_left_deg",60.0);'
    'declare_parameter<double>("package_scan_right_deg",60.0);'
    'declare_parameter<double>("package_scan_step_deg",60.0);'
    'declare_parameter<double>("package_scan_wait_sec",0.05);'
    'declare_parameter<double>("package_scan_timeout_sec",60.0);'
    'declare_parameter<double>("package_scan_wz_degps",3.2);'
    'declare_parameter<double>("package_scan_x_speed",5.6);'
    'declare_parameter<double>("package_scan_y_speed",5.6);'
    'declare_parameter<double>("package_scan_wz_speed",3.2);'
)

# 删除已有 scan declare，避免重复
for k in [
    "package_scan_range_deg",
    "package_scan_left_deg",
    "package_scan_right_deg",
    "package_scan_step_deg",
    "package_scan_wait_sec",
    "package_scan_timeout_sec",
    "package_scan_wz_degps",
    "package_scan_x_speed",
    "package_scan_y_speed",
    "package_scan_wz_speed",
]:
    s = re.sub(
        r'declare_parameter<double>\("' + re.escape(k) + r'",[^;]*;',
        '',
        s,
    )

# 插到 oak_align_min_wz 后面
s = re.sub(
    r'(declare_parameter<double>\("oak_align_min_wz",[^;]*;)'
    ,
    r'\1' + declare_block,
    s,
    count=1,
)

# ------------------------------------------------------------
# 4. get_parameter 强制写入
# ------------------------------------------------------------
get_block = (
    'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();'
    'ctx_->package_scan_left_deg=get_parameter("package_scan_left_deg").as_double();'
    'ctx_->package_scan_right_deg=get_parameter("package_scan_right_deg").as_double();'
    'ctx_->package_scan_step_deg=get_parameter("package_scan_step_deg").as_double();'
    'ctx_->package_scan_wait_sec=get_parameter("package_scan_wait_sec").as_double();'
    'ctx_->package_scan_timeout_sec=get_parameter("package_scan_timeout_sec").as_double();'
    'ctx_->package_scan_wz_degps=get_parameter("package_scan_wz_degps").as_double();'
    'ctx_->package_scan_x_speed=get_parameter("package_scan_x_speed").as_double();'
    'ctx_->package_scan_y_speed=get_parameter("package_scan_y_speed").as_double();'
    'ctx_->package_scan_wz_speed=get_parameter("package_scan_wz_speed").as_double();'
)

for k in [
    "package_scan_range_deg",
    "package_scan_left_deg",
    "package_scan_right_deg",
    "package_scan_step_deg",
    "package_scan_wait_sec",
    "package_scan_timeout_sec",
    "package_scan_wz_degps",
    "package_scan_x_speed",
    "package_scan_y_speed",
    "package_scan_wz_speed",
]:
    s = re.sub(
        r'ctx_->' + re.escape(k) + r'\s*=\s*get_parameter\("' + re.escape(k) + r'"\)\.as_double\(\);',
        '',
        s,
    )

s = re.sub(
    r'(ctx_->oak_align_min_wz\s*=\s*get_parameter\("oak_align_min_wz"\)\.as_double\(\);)',
    r'\1' + get_block,
    s,
    count=1,
)

# ------------------------------------------------------------
# 5. 替换 ScanTargetPackageOnShelf::build_offsets()
# ------------------------------------------------------------
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

        // 强制左右各 60 度，动作明显。
        ctx->package_scan_left_deg=60.0;
        ctx->package_scan_right_deg=60.0;
        ctx->package_scan_step_deg=60.0;
        ctx->package_scan_wz_degps=3.2;
        ctx->package_scan_x_speed=5.6;
        ctx->package_scan_y_speed=5.6;
        ctx->package_scan_wz_speed=3.2;

        // 扫描顺序：中心 -> 左 60 -> 右 60
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

# ------------------------------------------------------------
# 6. 只在 ScanTargetPackageOnShelf 内强制速度
# ------------------------------------------------------------
cls_pos = s.find("class ScanTargetPackageOnShelf")
cls_open = s.find("{", cls_pos)
cls_close = find_matching_brace(s, cls_open)
cls_text = s[cls_pos:cls_close + 1]

# 替换状态提示
cls_text = re.sub(
    r'ctx->status\("RUNNING",[^;]*?scanning target package Data Matrix on shelf[^;]*?\);',
    (
        'ctx->status("RUNNING",'
        '"asymmetric scanning target package Data Matrix on shelf: center=0deg, left=60deg, right=60deg, step=60deg, x=5.6, y=5.6, wz=3.2");'
    ),
    cls_text,
    count=1,
    flags=re.S,
)

# 替换所有扫描阶段 cmd.angular.z 赋值为 3.2 限幅
cls_text = re.sub(
    r'cmd\.angular\.z\s*=\s*clamp_abs_min\([^;]*?\);',
    (
        'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,3.2,0.2);'
    ),
    cls_text,
)

# 如果 ScanTargetPackageOnShelf 内有 linear.x/y 赋值，强制到 5.6
cls_text = re.sub(r'cmd\.linear\.x\s*=\s*[^;]*;', 'cmd.linear.x=5.6;', cls_text)
cls_text = re.sub(r'cmd\.linear\.y\s*=\s*[^;]*;', 'cmd.linear.y=5.6;', cls_text)

# 如果原来没有 linear.x/y，插到 publish 前面
if "cmd.linear.x=5.6;" not in cls_text:
    cls_text = cls_text.replace(
        "ctx->cmd_vel_pub->publish(cmd);",
        "cmd.linear.x=5.6;cmd.linear.y=5.6;ctx->cmd_vel_pub->publish(cmd);",
        1,
    )

s = s[:cls_pos] + cls_text + s[cls_close + 1:]

# ------------------------------------------------------------
# 7. 最终检查
# ------------------------------------------------------------
for c in [
    "class WaitNavSettle",
    "class RequestOakPackageSearchAlign",
    "class ScanTargetPackageOnShelf",
    "ctx->package_scan_left_deg=60.0",
    "ctx->package_scan_right_deg=60.0",
    "cmd.linear.x=5.6",
    "cmd.linear.y=5.6",
    "3.2",
]:
    if c not in s:
        raise RuntimeError(f"after patch missing: {c}")

CPP.write_text(s)

print("[OK] patched:", CPP)
print("[DONE] scan action forced:")
print("  offsets: 0 -60 +60")
print("  linear.x = 5.6")
print("  linear.y = 5.6")
print("  angular.z max = 3.2")
PY

echo "[3/6] Patch YAML configs too..."

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

params = {
    "package_scan_range_deg": "60.0",
    "package_scan_left_deg": "60.0",
    "package_scan_right_deg": "60.0",
    "package_scan_step_deg": "60.0",
    "package_scan_wait_sec": "0.05",
    "package_scan_timeout_sec": "60.0",
    "package_scan_wz_degps": "3.2",
    "package_scan_x_speed": "5.6",
    "package_scan_y_speed": "5.6",
    "package_scan_wz_speed": "3.2",
}

def backup(p):
    bak = p.with_suffix(p.suffix + f".bak_scan60_xy56_wz32_{STAMP}")
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

        # 只处理包含 mission 参数的文件
        if "package_scan" not in s and "mission_btcpp_node" not in s:
            continue

        for k, v in params.items():
            pat = r'(^\s*' + re.escape(k) + r'\s*:\s*)[-0-9.]+'
            if re.search(pat, s, flags=re.M):
                s = re.sub(pat, r'\g<1>' + v, s, flags=re.M)

        # 如果参数不存在，插到 package_scan_range_deg 后
        if "package_scan_range_deg" in s:
            lines = s.splitlines()
            existing = set()
            for line in lines:
                m = re.match(r'\s*([A-Za-z0-9_]+)\s*:', line)
                if m:
                    existing.add(m.group(1))

            out = []
            inserted = False
            for line in lines:
                out.append(line)
                if line.strip().startswith("package_scan_range_deg:") and not inserted:
                    indent = line[:len(line) - len(line.lstrip())]
                    for k, v in params.items():
                        if k not in existing:
                            out.append(f"{indent}{k}: {v}")
                    inserted = True

            s = "\n".join(out) + "\n"

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
echo "[CHECK] C++ scan forced values:"
grep -n "offsets deg\|package_scan_left_deg=60.0\|package_scan_right_deg=60.0\|cmd.linear.x=5.6\|cmd.linear.y=5.6\|3.2" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] YAML scan params:"
grep -R -n "package_scan_left_deg\|package_scan_right_deg\|package_scan_step_deg\|package_scan_x_speed\|package_scan_y_speed\|package_scan_wz_speed\|package_scan_wz_degps" \
/host-home/src/Navigation/robot_logistics_task/config \
/host-home/install/robot_logistics_task/share/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[DONE] Now restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
