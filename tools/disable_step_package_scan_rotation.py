#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config/mission_task.yaml"),
]

PATCH = {
    "package_scan_range_deg": 0.0,
    "package_scan_step_deg": 30.0,
    "package_scan_wait_sec": 0.0,
    "package_scan_timeout_sec": 3.0,
}


def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_disable_step_scan_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def patch_obj(obj):
    changed = False

    if isinstance(obj, dict):
        for k, v in PATCH.items():
            if k in obj and obj.get(k) != v:
                obj[k] = v
                changed = True

        if "ros__parameters" in obj and isinstance(obj["ros__parameters"], dict):
            params = obj["ros__parameters"]
            for k, v in PATCH.items():
                if k in params and params.get(k) != v:
                    params[k] = v
                    changed = True

        for v in obj.values():
            if patch_obj(v):
                changed = True

    elif isinstance(obj, list):
        for item in obj:
            if patch_obj(item):
                changed = True

    return changed


def patch_file(p: Path):
    if not p.exists():
        print(f"[SKIP] missing: {p}")
        return False

    data = yaml.safe_load(p.read_text(errors="ignore"))

    if not isinstance(data, dict):
        print(f"[SKIP] not yaml dict: {p}")
        return False

    if not patch_obj(data):
        print(f"[NO CHANGE] {p}")
        return False

    backup(p)
    p.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )
    print(f"[OK] patched: {p}")
    return True


def main():
    count = 0
    for p in FILES:
        if patch_file(p):
            count += 1

    print("")
    print("[DONE] disabled step-style package scan rotation")
    for k, v in PATCH.items():
        print(f"  {k}: {v}")
    print(f"[DONE] patched files: {count}")


if __name__ == "__main__":
    main()
