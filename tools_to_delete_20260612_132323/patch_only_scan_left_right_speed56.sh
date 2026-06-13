#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Patch ScanTargetPackageOnShelf only..."

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

# 按“扫描角速度 5.6 度/秒”处理
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


def count_class(s: str, name: str) -> int:
    return len(re.findall(r"class\s+" + re.escape(name) + r"\s*:", s))


def patch_cpp():
    if not CPP.exists():
        raise FileNotFoundError(CPP)

    s = CPP.read_text(errors="ignore")
    old = s

    # ------------------------------------------------------------
    # 安全检查：当前必须是已经恢复正常的版本
    # ------------------------------------------------------------
    required = [
        "class WaitNavSettle",
        "class RequestOakPackageSearchAlign",
        "class ScanTargetPackageOnShelf",
        "factory_.registerNodeType<RequestOakPackageSearchAlign",
    ]

    for r in required:
        if r not in s:
            raise RuntimeError(f"current C++ missing required stable part: {r}")

    if count_class(s, "WaitNavSettle") != 1:
        raise RuntimeError(
            f"WaitNavSettle class count is {count_class(s, 'WaitNavSettle')}, expected 1. "
            "先不要继续改扫描，当前 C++ 结构还不干净。"
        )

    backup(CPP, "before_patch_scan_left_right_speed56")

    # ------------------------------------------------------------
    # 1. SharedContext：统一 package_scan 参数
    # ------------------------------------------------------------
    scan_decl_pat = (
        r"double\s+package_scan_range_deg\s*=\s*[^;]*?"
        r"package_search_timeout_sec\s*=\s*[^;]*?;"
    )

    scan_decl_new = (
        "double package_scan_range_deg=12.0,"
        "package_scan_left_deg=12.0,"
        "package_scan_right_deg=8.0,"
        "package_scan_step_deg=4.0,"
        "package_scan_wait_sec=0.05,"
        "package_scan_timeout_sec=15.0,"
        "package_scan_wz_degps=5.6,"
        "package_search_timeout_sec=20.0;"
    )

    s2, n = re.subn(scan_decl_pat, scan_decl_new, s, count=1, flags=re.S)

    if n == 0:
        raise RuntimeError("cannot find package_scan parameter declaration in SharedContext")

    s = s2

    # ------------------------------------------------------------
    # 2. declare_parameter：先删除旧 left/right/wz，再插入新的
    # ------------------------------------------------------------
    for param in [
        "package_scan_left_deg",
        "package_scan_right_deg",
        "package_scan_wz_degps",
    ]:
        s = re.sub(
            r'declare_parameter<double>\("' + re.escape(param) + r'",[^;]*;',
            "",
            s,
        )

    s = re.sub(
        r'declare_parameter<double>\("package_scan_range_deg",[^;]*;',
        (
            'declare_parameter<double>("package_scan_range_deg",12.0);'
            'declare_parameter<double>("package_scan_left_deg",12.0);'
            'declare_parameter<double>("package_scan_right_deg",8.0);'
        ),
        s,
        count=1,
    )

    s = re.sub(
        r'declare_parameter<double>\("package_scan_step_deg",[^;]*;',
        'declare_parameter<double>("package_scan_step_deg",4.0);',
        s,
        count=1,
    )

    s = re.sub(
        r'declare_parameter<double>\("package_scan_wait_sec",[^;]*;',
        'declare_parameter<double>("package_scan_wait_sec",0.05);',
        s,
        count=1,
    )

    s = re.sub(
        r'declare_parameter<double>\("package_scan_timeout_sec",[^;]*;',
        (
            'declare_parameter<double>("package_scan_timeout_sec",15.0);'
            'declare_parameter<double>("package_scan_wz_degps",5.6);'
        ),
        s,
        count=1,
    )

    # ------------------------------------------------------------
    # 3. get_parameter：先删除旧 left/right/wz，再插入新的
    # ------------------------------------------------------------
    for param in [
        "package_scan_left_deg",
        "package_scan_right_deg",
        "package_scan_wz_degps",
    ]:
        s = re.sub(
            r'ctx_->' + re.escape(param) + r'\s*=\s*get_parameter\("' + re.escape(param) + r'"\)\.as_double\(\);',
            "",
            s,
        )

    s = re.sub(
        r'ctx_->package_scan_range_deg\s*=\s*get_parameter\("package_scan_range_deg"\)\.as_double\(\);',
        (
            'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();'
            'ctx_->package_scan_left_deg=get_parameter("package_scan_left_deg").as_double();'
            'ctx_->package_scan_right_deg=get_parameter("package_scan_right_deg").as_double();'
        ),
        s,
        count=1,
    )

    s = re.sub(
        r'ctx_->package_scan_step_deg\s*=\s*get_parameter\("package_scan_step_deg"\)\.as_double\(\);',
        'ctx_->package_scan_step_deg=get_parameter("package_scan_step_deg").as_double();',
        s,
        count=1,
    )

    s = re.sub(
        r'ctx_->package_scan_wait_sec\s*=\s*get_parameter\("package_scan_wait_sec"\)\.as_double\(\);',
        'ctx_->package_scan_wait_sec=get_parameter("package_scan_wait_sec").as_double();',
        s,
        count=1,
    )

    s = re.sub(
        r'ctx_->package_scan_timeout_sec\s*=\s*get_parameter\("package_scan_timeout_sec"\)\.as_double\(\);',
        (
            'ctx_->package_scan_timeout_sec=get_parameter("package_scan_timeout_sec").as_double();'
            'ctx_->package_scan_wz_degps=get_parameter("package_scan_wz_degps").as_double();'
        ),
        s,
        count=1,
    )

    # ------------------------------------------------------------
    # 4. 只修改 ScanTargetPackageOnShelf 类内部
    # ------------------------------------------------------------
    cls_pos = s.find("class ScanTargetPackageOnShelf")
    if cls_pos < 0:
        raise RuntimeError("cannot find class ScanTargetPackageOnShelf")

    cls_open = s.find("{", cls_pos)
    cls_close = find_matching_brace(s, cls_open)
    cls_text = s[cls_pos:cls_close + 1]

    # 4.1 替换 build_offsets()
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
        if(step<0.1)step=0.1;

        // OAK 对正后的货架正面方向 = 中心 0°
        // 左右扫顺序：0, -step, +step, -2step, +2step ...
        // left/right 可分别配置，不再强制左右对称。
        offsets_.push_back(0.0);

        double max_lr=(left>right)?left:right;
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

    # 重新取 ScanTargetPackageOnShelf 类文本
    cls_pos = s.find("class ScanTargetPackageOnShelf")
    cls_open = s.find("{", cls_pos)
    cls_close = find_matching_brace(s, cls_open)
    cls_text = s[cls_pos:cls_close + 1]

    # 4.2 修改扫描状态提示
    cls_text_new = cls_text.replace(
        'ctx->status("RUNNING","linear scanning target package Data Matrix on shelf");',
        (
            '{std::stringstream ss;'
            'ss<<"asymmetric scanning target package Data Matrix on shelf: center=0deg"'
            '<<", left="<<ctx->package_scan_left_deg<<"deg"'
            '<<", right="<<ctx->package_scan_right_deg<<"deg"'
            '<<", step="<<ctx->package_scan_step_deg<<"deg"'
            '<<", wz="<<ctx->package_scan_wz_degps<<"deg/s";'
            'ctx->status("RUNNING",ss.str());}'
        )
    )

    cls_text_new = cls_text_new.replace(
        '"target package not found in linear scan range"',
        '"target package not found in asymmetric scan range"'
    )

    # 4.3 只在 ScanTargetPackageOnShelf 内，把角速度限制改成 package_scan_wz_degps
    old_speed_line = 'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,ctx->oak_align_max_wz,ctx->oak_align_min_wz);'

    new_speed_block = (
        '{'
        'double max_wz=std::abs(ctx->package_scan_wz_degps)*M_PI/180.0;'
        'if(max_wz<0.02)max_wz=0.02;'
        'double min_wz=(max_wz<0.04)?max_wz:0.04;'
        'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,max_wz,min_wz);'
        '}'
    )

    if old_speed_line in cls_text_new:
        cls_text_new = cls_text_new.replace(old_speed_line, new_speed_block, 1)
    elif "package_scan_wz_degps" not in cls_text_new:
        raise RuntimeError("cannot find scan angular speed line inside ScanTargetPackageOnShelf")

    s = s[:cls_pos] + cls_text_new + s[cls_close + 1:]

    # ------------------------------------------------------------
    # 5. 检查这次没有动坏对正节点
    # ------------------------------------------------------------
    checks = [
        "class WaitNavSettle",
        "class RequestOakPackageSearchAlign",
        "factory_.registerNodeType<RequestOakPackageSearchAlign",
        "class ScanTargetPackageOnShelf",
        "package_scan_left_deg",
        "package_scan_right_deg",
        "package_scan_wz_degps",
    ]

    for c in checks:
        if c not in s:
            raise RuntimeError(f"after patch missing: {c}")

    if count_class(s, "WaitNavSettle") != 1:
        raise RuntimeError("after patch WaitNavSettle class count is not 1")

    if s == old:
        print("[NO CHANGE] C++ unchanged")
    else:
        CPP.write_text(s)
        print(f"[OK] patched C++: {CPP}")


def patch_yaml_file(p: Path):
    if not p.exists():
        print(f"[SKIP] missing yaml: {p}")
        return

    if yaml is None:
        print("[WARN] PyYAML not available, skip yaml:", p)
        return

    data = yaml.safe_load(p.read_text(errors="ignore"))
    if not isinstance(data, dict):
        print(f"[SKIP] yaml not dict: {p}")
        return

    changed = False

    def walk(obj):
        nonlocal changed

        if isinstance(obj, dict):
            if "ros__parameters" in obj and isinstance(obj["ros__parameters"], dict):
                params = obj["ros__parameters"]

                # 只改 mission_btcpp_node 这一类参数块
                if (
                    "bt_xml" in params
                    or "task_status_topic" in params
                    or "package_scan_range_deg" in params
                    or "package_search_request_topic" in params
                ):
                    desired = {
                        "package_scan_range_deg": max(LEFT_DEG, RIGHT_DEG),
                        "package_scan_left_deg": LEFT_DEG,
                        "package_scan_right_deg": RIGHT_DEG,
                        "package_scan_step_deg": STEP_DEG,
                        "package_scan_wait_sec": WAIT_SEC,
                        "package_scan_timeout_sec": TIMEOUT_SEC,
                        "package_scan_wz_degps": SCAN_WZ_DEGPS,
                    }

                    for k, v in desired.items():
                        if params.get(k) != v:
                            params[k] = v
                            changed = True

            for v in obj.values():
                walk(v)

        elif isinstance(obj, list):
            for item in obj:
                walk(item)

    walk(data)

    if changed:
        backup(p, "before_patch_scan_left_right_yaml")
        p.write_text(
            yaml.safe_dump(
                data,
                sort_keys=False,
                allow_unicode=True,
                default_flow_style=False,
            )
        )
        print(f"[OK] patched YAML: {p}")
    else:
        print(f"[NO CHANGE] YAML: {p}")


def main():
    patch_cpp()

    for p in YAML_FILES:
        patch_yaml_file(p)

    print("")
    print("[DONE] Only ScanTargetPackageOnShelf was changed.")
    print(f"  left_deg      = {LEFT_DEG}")
    print(f"  right_deg     = {RIGHT_DEG}")
    print(f"  step_deg      = {STEP_DEG}")
    print(f"  wait_sec      = {WAIT_SEC}")
    print(f"  timeout_sec   = {TIMEOUT_SEC}")
    print(f"  scan_wz_degps = {SCAN_WZ_DEGPS}")
    print("")
    print("Scan order should be:")
    print("  0 -> -4 -> +4 -> -8 -> +8 -> -12")


if __name__ == "__main__":
    main()
PY


echo "[3/6] Clean rebuild robot_logistics_task and robot2_bringup..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash

colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[4/6] Source install..."
source /host-home/install/setup.bash

echo "[5/6] Check patched params and code..."
echo ""
echo "[CHECK] C++ scan params:"
grep -n "package_scan_left_deg\|package_scan_right_deg\|package_scan_wz_degps\|asymmetric scanning\|offsets deg" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] Do not touch OAK align nodes:"
grep -n "class WaitNavSettle\|class RequestOakPackageSearchAlign\|registerNodeType<RequestOakPackageSearchAlign" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[6/6] Done."
echo "Now restart with:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
