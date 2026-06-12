#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Patch C++ scan step safety and YAML params..."

python3 - <<'PY'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re

try:
    import yaml
except Exception:
    yaml = None

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

YAML_FILES = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config/mission_task.yaml"),
]

LEFT_DEG = 12.0
RIGHT_DEG = 8.0
STEP_DEG = 4.0
WAIT_SEC = 0.05
TIMEOUT_SEC = 15.0
SCAN_WZ_DEGPS = 5.6


def backup(p: Path, tag: str):
    if not p.exists():
        return
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


def patch_cpp():
    if not CPP.exists():
        raise FileNotFoundError(CPP)

    s = CPP.read_text(errors="ignore")
    old = s

    if "class ScanTargetPackageOnShelf" not in s:
        raise RuntimeError("cannot find ScanTargetPackageOnShelf")

    backup(CPP, "before_fix_scan_step_offsets")

    # 1. C++ 默认值改成 4，不再是 30
    s = re.sub(
        r'package_scan_step_deg\s*=\s*[-0-9.]+',
        'package_scan_step_deg=4.0',
        s,
        count=1,
    )

    s = re.sub(
        r'declare_parameter<double>\("package_scan_step_deg",\s*[-0-9.]+\s*\);',
        'declare_parameter<double>("package_scan_step_deg",4.0);',
        s,
    )

    # 2. 只改 ScanTargetPackageOnShelf 里面 build_offsets 的保险逻辑
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
        double step=ctx->package_scan_step_deg;

        if(left<0.0)left=0.0;
        if(right<0.0)right=0.0;

        double max_lr=(left>right)?left:right;

        // 保险：如果 YAML/参数误把 step 设成 30deg，而 left/right 只有 12/8deg，
        // 就会只生成 0deg，导致完全不左右扫。
        // 所以这里强制把异常 step 压回 4deg。
        if(step<0.1 || step>max_lr+1e-6){
            step=4.0;
        }

        if(step<0.1){
            step=0.1;
        }

        // OAK 对正后的货架正面方向 = 中心 0°
        // 左右扫顺序：0, -step, +step, -2step, +2step ...
        offsets_.push_back(0.0);

        for(double d=step;d<=max_lr+1e-6;d+=step){
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

    if s != old:
        CPP.write_text(s)
        print(f"[OK] patched C++: {CPP}")
    else:
        print("[NO CHANGE] C++")


def patch_yaml_text_simple(p: Path):
    if not p.exists():
        print(f"[SKIP] missing yaml: {p}")
        return

    s = p.read_text(errors="ignore")
    old = s

    # 先用正则直接修所有已有参数
    replacements = {
        "package_scan_range_deg": "12.0",
        "package_scan_left_deg": "12.0",
        "package_scan_right_deg": "8.0",
        "package_scan_step_deg": "4.0",
        "package_scan_wait_sec": "0.05",
        "package_scan_timeout_sec": "15.0",
        "package_scan_wz_degps": "5.6",
    }

    for k, v in replacements.items():
        if re.search(r'(^\s*' + re.escape(k) + r'\s*:\s*)[-0-9.]+', s, flags=re.M):
            s = re.sub(
                r'(^\s*' + re.escape(k) + r'\s*:\s*)[-0-9.]+',
                r'\g<1>' + v,
                s,
                flags=re.M,
            )

    # 如果 YAML 有 range 但没有 left/right/wz，就插到 range 后面
    if "package_scan_range_deg" in s:
        lines = s.splitlines()
        out = []
        inserted = set()

        for line in lines:
            out.append(line)
            stripped = line.strip()

            if stripped.startswith("package_scan_range_deg:"):
                indent = line[:len(line) - len(line.lstrip())]
                for k, v in [
                    ("package_scan_left_deg", "12.0"),
                    ("package_scan_right_deg", "8.0"),
                ]:
                    if k not in s and k not in inserted:
                        out.append(f"{indent}{k}: {v}")
                        inserted.add(k)

            if stripped.startswith("package_scan_timeout_sec:"):
                indent = line[:len(line) - len(line.lstrip())]
                if "package_scan_wz_degps" not in s and "package_scan_wz_degps" not in inserted:
                    out.append(f"{indent}package_scan_wz_degps: 5.6")
                    inserted.add("package_scan_wz_degps")

        s = "\n".join(out) + "\n"

    if s != old:
        backup(p, "before_fix_scan_step_yaml")
        p.write_text(s)
        print(f"[OK] patched YAML: {p}")
    else:
        print(f"[NO CHANGE] YAML: {p}")


def main():
    patch_cpp()

    for p in YAML_FILES:
        patch_yaml_text_simple(p)

    print("")
    print("[DONE] scan step fixed")
    print("Expected:")
    print("  left=12deg, right=8deg, step=4deg, wz=5.6deg/s")
    print("  offsets: 0 -4 4 -8 8 -12")


if __name__ == "__main__":
    main()
PY

echo "[3/6] Clean rebuild..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[4/6] Source install..."
source /host-home/install/setup.bash

echo "[5/6] Check active config..."
echo ""
echo "[CHECK] source YAML:"
grep -R -n "package_scan_left_deg\|package_scan_right_deg\|package_scan_step_deg\|package_scan_wz_degps" \
/host-home/src/Navigation/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] install YAML:"
grep -R -n "package_scan_left_deg\|package_scan_right_deg\|package_scan_step_deg\|package_scan_wz_degps" \
/host-home/install/robot_logistics_task/share/robot_logistics_task/config 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] C++:"
grep -n "package_scan_step_deg\|offsets deg\|step>max_lr" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[6/6] Done."
echo "Now restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
