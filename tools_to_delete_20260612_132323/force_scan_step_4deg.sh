#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Force ScanTargetPackageOnShelf step to 4deg in C++..."

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
old = s

if "class ScanTargetPackageOnShelf" not in s:
    raise RuntimeError("cannot find class ScanTargetPackageOnShelf")

if "class RequestOakPackageSearchAlign" not in s:
    raise RuntimeError("RequestOakPackageSearchAlign missing, do not continue")

if "class WaitNavSettle" not in s:
    raise RuntimeError("WaitNavSettle missing, do not continue")

backup(CPP, "before_force_scan_step_4deg")

# 1. 全局默认参数也改成 4，防止后续被默认值污染
s = re.sub(
    r'package_scan_step_deg\s*=\s*[-0-9.]+',
    'package_scan_step_deg=4.0',
    s,
)

s = re.sub(
    r'declare_parameter<double>\("package_scan_step_deg",\s*[-0-9.]+\s*\);',
    'declare_parameter<double>("package_scan_step_deg",4.0);',
    s,
)

# 2. 只替换 ScanTargetPackageOnShelf::build_offsets()
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

        double left=ctx->package_scan_left_deg;
        double right=ctx->package_scan_right_deg;

        if(left<0.0)left=0.0;
        if(right<0.0)right=0.0;

        // 强制固定 4deg，避免 YAML 里 30deg 覆盖后只生成中心 0deg。
        double step=4.0;
        ctx->package_scan_step_deg=4.0;

        // OAK 对正后的货架正面方向 = 中心 0°
        // 左右扫顺序：0, -4, +4, -8, +8, -12
        offsets_.push_back(0.0);

        double max_lr=(left>right)?left:right;
        for(double d=step; d<=max_lr+1e-6; d+=step){
            if(d<=left+1e-6){
                offsets_.push_back(-d*M_PI/180.0);
            }
            if(d<=right+1e-6){
                offsets_.push_back(d*M_PI/180.0);
            }
        }

        std::stringstream ss;
        ss<<"[ScanPackage] offsets deg:";
        for(double v:offsets_){
            ss<<" "<<(v*180.0/M_PI);
        }
        RCLCPP_INFO(ctx->node->get_logger(),"%s",ss.str().c_str());
    }'''

s = s[:fn_open] + new_build_offsets + s[fn_close + 1:]

# 3. 强制把 status 里的 step 显示为 4，避免你看日志被旧参数误导
s = s.replace(
    '<<", step="<<ctx->package_scan_step_deg<<"deg"',
    '<<", step="<<4.0<<"deg"'
)

CPP.write_text(s)

print("[OK] patched C++:", CPP)
print("[DONE] forced scan step to 4deg")
print("Expected offsets: 0 -4 4 -8 8 -12")
PY

echo "[3/6] Patch all active YAML configs to step=4..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src"),
    Path("/host-home/install"),
]

params = {
    "package_scan_left_deg": "12.0",
    "package_scan_right_deg": "8.0",
    "package_scan_step_deg": "4.0",
    "package_scan_wz_degps": "5.6",
    "package_scan_wait_sec": "0.05",
    "package_scan_timeout_sec": "15.0",
}

def backup(p):
    bak = p.with_suffix(p.suffix + f".bak_force_scan_step_4deg_{STAMP}")
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
        if p.suffix not in [".yaml", ".yml", ".py", ".launch"]:
            continue

        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue

        if "package_scan_step_deg" not in s and "package_scan_range_deg" not in s:
            continue

        old = s

        for k, v in params.items():
            pat = r'(^\s*' + re.escape(k) + r'\s*:\s*)[-0-9.]+'
            if re.search(pat, s, flags=re.M):
                s = re.sub(pat, r'\g<1>' + v, s, flags=re.M)

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
echo "[CHECK] C++ must show forced step:"
grep -n "强制固定 4deg\|ctx->package_scan_step_deg=4.0\|offsets deg" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] configs containing package_scan_step_deg:"
grep -R -n "package_scan_step_deg" /host-home/src /host-home/install 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[DONE] Now restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
