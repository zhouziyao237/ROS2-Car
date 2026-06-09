#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import shutil

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
XML_DIR = Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree")


def backup(p: Path):
    if p.exists():
        b = p.with_suffix(p.suffix + f".bak_settle_wait_{STAMP}")
        shutil.copy2(p, b)
        print(f"[BACKUP] {p} -> {b}")


def patch_cpp():
    if not CPP.exists():
        raise SystemExit(f"[ERROR] not found: {CPP}")

    backup(CPP)
    s = CPP.read_text(errors="ignore")
    changed = False

    if "class WaitNavSettle" not in s:
        old = "class WaitNavArrive:public TimedAction{public:WaitNavArrive(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,\"WaitNavArrive\",2.0){}SIMPLE_PORTS};"
        new = old + "\n" + \
'''class WaitNavSettle:public TimedAction{public:WaitNavSettle(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"WaitNavSettle",3.0){}SIMPLE_PORTS};
class WaitAfterOakAlign:public TimedAction{public:WaitAfterOakAlign(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"WaitAfterOakAlign",1.5){}SIMPLE_PORTS};'''
        if old not in s:
            raise SystemExit("[ERROR] cannot find WaitNavArrive class insertion point")
        s = s.replace(old, new, 1)
        changed = True
        print("[OK] added WaitNavSettle and WaitAfterOakAlign classes")

    if "registerNodeType<WaitNavSettle>" not in s:
        old = 'factory_.registerNodeType<WaitNavArrive>("WaitNavArrive");'
        new = old + 'factory_.registerNodeType<WaitNavSettle>("WaitNavSettle");factory_.registerNodeType<WaitAfterOakAlign>("WaitAfterOakAlign");'
        if old not in s:
            raise SystemExit("[ERROR] cannot find WaitNavArrive register point")
        s = s.replace(old, new, 1)
        changed = True
        print("[OK] registered WaitNavSettle and WaitAfterOakAlign")

    if changed:
        CPP.write_text(s)
        print(f"[OK] wrote {CPP}")
    else:
        print("[SKIP] C++ already patched")


def patch_xml():
    if not XML_DIR.exists():
        raise SystemExit(f"[ERROR] not found: {XML_DIR}")

    for p in sorted(XML_DIR.glob("*.xml")):
        s = p.read_text(errors="ignore")
        original = s

        if "<WaitNavArrive/>" in s and "<WaitNavSettle/>" not in s:
            s = s.replace(
                "<WaitNavArrive/>",
                "<WaitNavArrive/>\n      <!-- 等导航停止、速度归零、TF 稳定后再进入货架验证和 OAK 对正 -->\n      <WaitNavSettle/>",
                1,
            )

        if "<RequestOakPackageSearchAlign/>" in s and "<WaitAfterOakAlign/>" not in s:
            s = s.replace(
                "<RequestOakPackageSearchAlign/>",
                "<RequestOakPackageSearchAlign/>\n      <!-- OAK 对正完成后稍等，再进入后续扫描/平台动作 -->\n      <WaitAfterOakAlign/>",
                1,
            )

        if s != original:
            backup(p)
            p.write_text(s)
            print(f"[OK] patched XML waits: {p}")
        else:
            print(f"[SKIP] no XML change needed: {p}")


def main():
    print("============================================================")
    print("[PATCH] add settle waits after navigation and OAK alignment")
    print("============================================================")
    patch_cpp()
    patch_xml()
    print("============================================================")
    print("[DONE]")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot_logistics_task")
    print("  source install/setup.bash")
    print("  pkill -f 'mission_btcpp_node|bt_osd_overlay' || true")
    print("  ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task")
    print("============================================================")


if __name__ == "__main__":
    main()
