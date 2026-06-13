#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re
import yaml

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
TIMEOUT_SEC = 12.0

# 这里按“扫描旋转速度 5.6 度/秒”处理
SCAN_WZ_DEGPS = 5.6


def backup(p: Path, tag: str):
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def restore_good_cpp():
    # 优先恢复第一次坏补丁之前的备份
    candidates = sorted(
        CPP.parent.glob("mission_btcpp_node.cpp.bak_left_right_scan_cpp_*"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        print("[WARN] no bak_left_right_scan_cpp backup found, patch current file directly")
        return

    src = candidates[0]
    print(f"[RESTORE] cpp from {src}")
    backup(CPP, "before_restore_bad_cpp")
    shutil.copy2(src, CPP)


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

    if "class RequestOakPackageSearchAlign" not in s or "class VerifyShelfQR" not in s:
        raise RuntimeError("CPP structure is still broken: RequestOakPackageSearchAlign or VerifyShelfQR missing")

    # 1. SharedContext 里增加左右扫描角度和扫描速度参数
    s = re.sub(
        r"double\s+package_scan_range_deg\s*=\s*[^;]*?package_search_timeout_sec\s*=\s*[^;]*?;",
        (
            "double package_scan_range_deg=25.0,"
            "package_scan_left_deg=12.0,"
            "package_scan_right_deg=8.0,"
            "package_scan_step_deg=4.0,"
            "package_scan_wait_sec=0.05,"
            "package_scan_timeout_sec=12.0,"
            "package_scan_wz_degps=5.6,"
            "package_search_timeout_sec=20.0;"
        ),
        s,
        count=1,
    )

    # 2. declare_parameter 增加新参数
    s = re.sub(
        r'declare_parameter<double>\("package_scan_range_deg",[^;]*;'
        r'declare_parameter<double>\("package_scan_step_deg",[^;]*;'
        r'declare_parameter<double>\("package_scan_wait_sec",[^;]*;'
        r'declare_parameter<double>\("package_scan_timeout_sec",[^;]*;',
        (
            'declare_parameter<double>("package_scan_range_deg",25.0);'
            'declare_parameter<double>("package_scan_left_deg",12.0);'
            'declare_parameter<double>("package_scan_right_deg",8.0);'
            'declare_parameter<double>("package_scan_step_deg",4.0);'
            'declare_parameter<double>("package_scan_wait_sec",0.05);'
            'declare_parameter<double>("package_scan_timeout_sec",12.0);'
            'declare_parameter<double>("package_scan_wz_degps",5.6);'
        ),
        s,
        count=1,
    )

    # 3. get_parameter 读取新参数
    s = re.sub(
        r'ctx_->package_scan_range_deg=get_parameter\("package_scan_range_deg"\)\.as_double\(\);'
        r'ctx_->package_scan_step_deg=get_parameter\("package_scan_step_deg"\)\.as_double\(\);'
        r'ctx_->package_scan_wait_sec=get_parameter\("package_scan_wait_sec"\)\.as_double\(\);'
        r'ctx_->package_scan_timeout_sec=get_parameter\("package_scan_timeout_sec"\)\.as_double\(\);'
        r'ctx_->package_search_timeout_sec=get_parameter\("package_search_timeout_sec"\)\.as_double\(\);',
        (
            'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();'
            'ctx_->package_scan_left_deg=get_parameter("package_scan_left_deg").as_double();'
            'ctx_->package_scan_right_deg=get_parameter("package_scan_right_deg").as_double();'
            'ctx_->package_scan_step_deg=get_parameter("package_scan_step_deg").as_double();'
            'ctx_->package_scan_wait_sec=get_parameter("package_scan_wait_sec").as_double();'
            'ctx_->package_scan_timeout_sec=get_parameter("package_scan_timeout_sec").as_double();'
            'ctx_->package_scan_wz_degps=get_parameter("package_scan_wz_degps").as_double();'
            'ctx_->package_search_timeout_sec=get_parameter("package_search_timeout_sec").as_double();'
        ),
        s,
        count=1,
    )

    # 4. 只替换 ScanTargetPackageOnShelf 里面的 build_offsets 函数体
    cls_pos = s.find("class ScanTargetPackageOnShelf")
    if cls_pos < 0:
        raise RuntimeError("cannot find class ScanTargetPackageOnShelf")

    cls_open = s.find("{", cls_pos)
    cls_close = find_matching_brace(s, cls_open)
    fn_pos = s.find("build_offsets", cls_open, cls_close)
    if fn_pos < 0:
        raise RuntimeError("cannot find build_offsets inside ScanTargetPackageOnShelf")

    fn_open = s.find("{", fn_pos)
    fn_close = find_matching_brace(s, fn_open)

    new_build_offsets = (
        "{\n"
        "        offsets_.clear();\n"
        "\n"
        "        double left=ctx->package_scan_left_deg;\n"
        "        double right=ctx->package_scan_right_deg;\n"
        "        double step=ctx->package_scan_step_deg;\n"
        "\n"
        "        if(left<0.0)left=0.0;\n"
        "        if(right<0.0)right=0.0;\n"
        "        if(step<0.1)step=0.1;\n"
        "\n"
        "        // 你的定义：OAK 对正后的货架正面方向 = 中心 0°。\n"
        "        // 扫描顺序：0, -step, +step, -2step, +2step ...\n"
        "        // left/right 可分别设置，不再强制左右对称。\n"
        "        offsets_.push_back(0.0);\n"
        "\n"
        "        double max_lr=(left>right)?left:right;\n"
        "        for(double d=step;d<=max_lr+1e-6;d+=step){\n"
        "            if(d<=left+1e-6)offsets_.push_back(-d*M_PI/180.0);\n"
        "            if(d<=right+1e-6)offsets_.push_back(d*M_PI/180.0);\n"
        "        }\n"
        "    }"
    )

    s = s[:fn_open] + new_build_offsets + s[fn_close + 1:]

    # 5. 状态提示改成 asymmetric scan
    s = s.replace(
        'ctx->status("RUNNING","linear scanning target package Data Matrix on shelf");',
        (
            '{std::stringstream ss;'
            'ss<<"asymmetric scanning target package Data Matrix on shelf: center=0deg"'
            '<<", left="<<ctx->package_scan_left_deg<<"deg"'
            '<<", right="<<ctx->package_scan_right_deg<<"deg"'
            '<<", step="<<ctx->package_scan_step_deg<<"deg"'
            '<<", wz="<<ctx->package_scan_wz_degps<<"deg/s";'
            'ctx->status("RUNNING",ss.str());}'
        ),
        1,
    )

    # 6. 扫描旋转速度单独用 package_scan_wz_degps，不再直接共用 oak_align_max_wz
    s = s.replace(
        'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,ctx->oak_align_max_wz,ctx->oak_align_min_wz);',
        (
            '{double max_wz=std::abs(ctx->package_scan_wz_degps)*M_PI/180.0;'
            'if(max_wz<0.02)max_wz=0.02;'
            'double min_wz=std::min(0.04,max_wz);'
            'cmd.angular.z=clamp_abs_min(ctx->oak_align_kp*err,max_wz,min_wz);}'
        ),
        1,
    )

    if s == old:
        print("[NO CHANGE] cpp")
        return False

    backup(CPP, "final_asymmetric_scan_speed56_cpp")
    CPP.write_text(s)
    print(f"[OK] patched cpp: {CPP}")
    return True


def patch_yaml_file(p: Path):
    if not p.exists():
        print(f"[SKIP] missing yaml: {p}")
        return False

    data = yaml.safe_load(p.read_text(errors="ignore"))
    if not isinstance(data, dict):
        print(f"[SKIP] not yaml dict: {p}")
        return False

    changed = False

    def walk(obj):
        nonlocal changed

        if isinstance(obj, dict):
            if "ros__parameters" in obj and isinstance(obj["ros__parameters"], dict):
                params = obj["ros__parameters"]

                if (
                    "package_scan_range_deg" in params
                    or "package_scan_step_deg" in params
                    or "bt_xml" in params
                    or "task_status_topic" in params
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

    if not changed:
        print(f"[NO CHANGE] yaml: {p}")
        return False

    backup(p, "final_asymmetric_scan_speed56_yaml")
    p.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True, default_flow_style=False))
    print(f"[OK] patched yaml: {p}")
    return True


def main():
    restore_good_cpp()
    cpp_changed = patch_cpp()

    yaml_count = 0
    for p in YAML_FILES:
        if patch_yaml_file(p):
            yaml_count += 1

    print("")
    print("[DONE] final asymmetric package scan applied")
    print("  center = OAK aligned shelf-front direction")
    print(f"  left_deg        = {LEFT_DEG}")
    print(f"  right_deg       = {RIGHT_DEG}")
    print(f"  step_deg        = {STEP_DEG}")
    print(f"  wait_sec        = {WAIT_SEC}")
    print(f"  timeout_sec     = {TIMEOUT_SEC}")
    print(f"  scan_wz_degps   = {SCAN_WZ_DEGPS}")
    print(f"  cpp_changed={cpp_changed}, yaml_changed={yaml_count}")


if __name__ == "__main__":
    main()
