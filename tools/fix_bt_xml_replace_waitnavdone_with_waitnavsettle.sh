#!/usr/bin/env bash
set -e

echo "[1/6] Stop current broken processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Replace WaitNavDoneBeforeOak with WaitNavSettle in active BT/YAML/XML files..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src"),
    Path("/host-home/install"),
]

TARGET = "WaitNavDoneBeforeOak"
REPLACEMENT = "WaitNavSettle"

SUFFIXES = {
    ".yaml", ".yml", ".xml", ".launch", ".py", ".txt"
}

def is_active_text_file(p: Path):
    if not p.is_file():
        return False
    if ".bak" in p.name or p.name.endswith("~") or p.name.endswith(".old"):
        return False
    if p.suffix in SUFFIXES:
        return True
    if "mission" in p.name or "bt" in p.name:
        return True
    return False

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_replace_waitnavdone_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

changed = []

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not is_active_text_file(p):
            continue

        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue

        if TARGET not in s:
            continue

        ns = s.replace(TARGET, REPLACEMENT)

        if ns != s:
            backup(p)
            p.write_text(ns)
            changed.append(p)
            print(f"[OK] {p}")

print("")
print("[DONE] changed files:")
for p in changed:
    print(" ", p)

print("")
print("[CHECK] Remaining active WaitNavDoneBeforeOak outside C++:")
left = []

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not is_active_text_file(p):
            continue

        # C++ 里保留类/注册没关系，真正崩的是 XML/YAML 引用
        if p.suffix in {".cpp", ".hpp", ".h"}:
            continue

        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue

        if TARGET in s:
            left.append(p)

if left:
    for p in left:
        print("[LEFT]", p)
    raise SystemExit("Still has WaitNavDoneBeforeOak in active BT/YAML/XML files.")
else:
    print("[OK] no active BT/YAML/XML still uses WaitNavDoneBeforeOak")
PY

echo "[3/6] Show active WaitNavSettle / RequestOakPackageSearchAlign references..."
grep -R -n "WaitNavSettle\|RequestOakPackageSearchAlign" \
/host-home/src/Navigation/robot_logistics_task \
/host-home/install/robot_logistics_task 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[4/6] Clean rebuild robot_logistics_task and robot2_bringup..."
cd /host-home

rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

source /opt/ros/humble/setup.bash

colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[5/6] Source install..."
source /host-home/install/setup.bash

echo "[6/6] Final checks..."

echo ""
echo "[CHECK] active BT/YAML/XML should NOT contain WaitNavDoneBeforeOak:"
grep -R -n "WaitNavDoneBeforeOak" \
/host-home/src/Navigation/robot_logistics_task \
/host-home/install/robot_logistics_task 2>/dev/null | grep -v ".bak" | grep -v "mission_btcpp_node.cpp" || true

echo ""
echo "[CHECK] binary should contain WaitNavSettle and RequestOakPackageSearchAlign:"
strings /host-home/install/robot_logistics_task/lib/robot_logistics_task/mission_btcpp_node | grep -E "WaitNavSettle|RequestOakPackageSearchAlign" || true

echo ""
echo "[DONE] BT XML mismatch fixed."
echo "Now restart with:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
