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

# 这里就是你定义的角度
SCAN_LEFT_DEG = 12.0
SCAN_RIGHT_DEG = 8.0
SCAN_STEP_DEG = 4.0
SCAN_WAIT_SEC = 0.05
SCAN_TIMEOUT_SEC = 12.0


def backup(p: Path, tag: str):
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def patch_cpp():
    if not CPP.exists():
        raise FileNotFoundError(CPP)

    s = CPP.read_text(errors="ignore")
    old = s

    # 1) SharedContext 增加 left/right 成员，保留旧 range 兼容
    s = re.sub(
        r"double\s+package_scan_range_deg\s*=\s*([^,;]+),\s*package_scan_step_deg",
        r"double package_scan_range_deg=\1,package_scan_left_deg=\1,package_scan_right_deg=\1,package_scan_step_deg",
        s,
        count=1,
    )

    # 2) declare_parameter 增加新参数
    if 'declare_parameter<double>("package_scan_left_deg"' not in s:
        s = s.replace(
            'declare_parameter<double>("package_scan_range_deg",25.0);',
            'declare_parameter<double>("package_scan_range_deg",25.0);'
            f'declare_parameter<double>("package_scan_left_deg",{SCAN_LEFT_DEG});'
            f'declare_parameter<double>("package_scan_right_deg",{SCAN_RIGHT_DEG});',
            1,
        )

    # 3) get_parameter 读取新参数
    if "ctx_->package_scan_left_deg=get_parameter" not in s:
        s = s.replace(
            'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();',
            'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();'
            'ctx_->package_scan_left_deg=get_parameter("package_scan_left_deg").as_double();'
            'ctx_->package_scan_right_deg=get_parameter("package_scan_right_deg").as_double();',
            1,
        )

    # 4) 修改 ScanTargetPackageOnShelf 的 offset 生成逻辑
    # 原逻辑是 range 对称扫描；这里改为 center-first + 左右独立角度。
    class_pat = re.compile(
        r"(class\s+ScanTargetPackageOnShelf\s*:\s*public\s+BT::StatefulActionNode\s*\{)(.*?)(\};\s*class\s+\w+)",
        re.S,
    )

    m = class_pat.search(s)
    if not m:
        print("[ERROR] cannot find class ScanTargetPackageOnShelf")
        print("请执行：")
        print("grep -n \"class ScanTargetPackageOnShelf\\|package_scan_range_deg\\|offsets_\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
        raise SystemExit(2)

    head, body, tail = m.group(1), m.group(2), m.group(3)

    if "package_scan_left_deg" not in body:
        # 找 offsets_.clear() 到 idx_=0 或 index_=0 之间的旧 offset 生成段
        offset_pat = re.compile(
            r"offsets_\.clear\(\);.*?((?:idx_|index_)\s*=\s*0\s*;)",
            re.S,
        )

        new_offset_code = (
            'offsets_.clear();'
            '{'
            'double step=ctx->package_scan_step_deg;'
            'if(step<0.1)step=0.1;'
            'double left=ctx->package_scan_left_deg;'
            'double right=ctx->package_scan_right_deg;'
            'if(left<0.0)left=0.0;'
            'if(right<0.0)right=0.0;'
            'offsets_.push_back(0.0);'
            'double max_lr=(left>right)?left:right;'
            'for(double d=step;d<=max_lr+1e-6;d+=step){'
            'if(d<=left+1e-6)offsets_.push_back(-d);'
            'if(d<=right+1e-6)offsets_.push_back(d);'
            '}'
            '}'
            r'\1'
        )

        body2, n = offset_pat.subn(new_offset_code, body, count=1)

        if n != 1:
            print("[ERROR] cannot patch offset generation inside ScanTargetPackageOnShelf")
            print("请执行：")
            print("grep -n \"offsets_\\|package_scan_range_deg\\|ScanPackage\\|index_\\|idx_\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            raise SystemExit(3)

        s = s[:m.start()] + head + body2 + tail + s[m.end():]

    if s == old:
        print("[NO CHANGE] C++ already patched")
        return False

    backup(CPP, "left_right_scan_cpp")
    CPP.write_text(s)
    print(f"[OK] patched C++: {CPP}")
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

    def patch_obj(obj):
        nonlocal changed
        if isinstance(obj, dict):
            if "ros__parameters" in obj and isinstance(obj["ros__parameters"], dict):
                params = obj["ros__parameters"]

                # 只改 mission_btcpp_node 相关参数块
                if (
                    "package_scan_range_deg" in params
                    or "package_scan_step_deg" in params
                    or "bt_xml" in params
                    or "task_status_topic" in params
                ):
                    desired = {
                        "package_scan_range_deg": max(SCAN_LEFT_DEG, SCAN_RIGHT_DEG),  # 旧参数保留兼容
                        "package_scan_left_deg": SCAN_LEFT_DEG,
                        "package_scan_right_deg": SCAN_RIGHT_DEG,
                        "package_scan_step_deg": SCAN_STEP_DEG,
                        "package_scan_wait_sec": SCAN_WAIT_SEC,
                        "package_scan_timeout_sec": SCAN_TIMEOUT_SEC,
                    }
                    for k, v in desired.items():
                        if params.get(k) != v:
                            params[k] = v
                            changed = True

            for v in obj.values():
                patch_obj(v)

        elif isinstance(obj, list):
            for item in obj:
                patch_obj(item)

    patch_obj(data)

    if not changed:
        print(f"[NO CHANGE] yaml: {p}")
        return False

    backup(p, "left_right_scan_yaml")
    p.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )
    print(f"[OK] patched yaml: {p}")
    return True


def main():
    cpp_changed = patch_cpp()

    yaml_count = 0
    for p in YAML_FILES:
        if patch_yaml_file(p):
            yaml_count += 1

    print("")
    print("[DONE] package scan now uses asymmetric left/right angles")
    print(f"[DONE] center = shelf front after OAK align")
    print(f"[DONE] left_deg  = {SCAN_LEFT_DEG}")
    print(f"[DONE] right_deg = {SCAN_RIGHT_DEG}")
    print(f"[DONE] step_deg  = {SCAN_STEP_DEG}")
    print(f"[DONE] wait_sec  = {SCAN_WAIT_SEC}")
    print(f"[DONE] timeout   = {SCAN_TIMEOUT_SEC}")
    print(f"[DONE] cpp_changed={cpp_changed}, yaml_changed={yaml_count}")


if __name__ == "__main__":
    main()
