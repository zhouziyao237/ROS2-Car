#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import sys

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = [
    Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_nav_oak_scan_only.xml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_nav_oak_scan_only.xml"),
]

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")


def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_wait_nav_done_runtime_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")


def patch_xml(path: Path):
    if not path.exists():
        print(f"[SKIP] missing: {path}")
        return False

    s = path.read_text(errors="ignore")

    if "WaitNavDoneBeforeOak" in s:
        print(f"[NO CHANGE] already has WaitNavDoneBeforeOak: {path}")
        return False

    if "<WaitNavArrive/>" not in s:
        print(f"[ERROR] cannot find <WaitNavArrive/> in {path}")
        return False

    s2 = s.replace(
        "<WaitNavArrive/>",
        "<WaitNavArrive/>\n      <WaitNavDoneBeforeOak/>",
        1
    )

    backup(path)
    path.write_text(s2)

    print(f"[OK] patched runtime BT XML: {path}")
    return True


def check_cpp_registered():
    if not CPP.exists():
        print(f"[WARN] missing C++ file: {CPP}")
        return

    s = CPP.read_text(errors="ignore")

    ok_class = "class WaitNavDoneBeforeOak" in s
    ok_register = 'registerNodeType<WaitNavDoneBeforeOak>("WaitNavDoneBeforeOak")' in s

    print("")
    print("[CHECK] C++ WaitNavDoneBeforeOak class:", "OK" if ok_class else "MISSING")
    print("[CHECK] C++ WaitNavDoneBeforeOak register:", "OK" if ok_register else "MISSING")

    if not ok_class or not ok_register:
        print("")
        print("[ERROR] C++ node is not registered. You need to rerun v2 patch and rebuild robot_logistics_task.")
        sys.exit(2)


def main():
    changed = 0

    check_cpp_registered()

    for f in FILES:
        if patch_xml(f):
            changed += 1

    print("")
    print(f"[DONE] patched XML files: {changed}")
    print("[DONE] Runtime BT should now be:")
    print("       WaitNavArrive -> WaitNavDoneBeforeOak -> RequestOakPackageSearchAlign")


if __name__ == "__main__":
    main()
