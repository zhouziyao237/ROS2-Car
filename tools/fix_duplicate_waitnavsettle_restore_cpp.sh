#!/usr/bin/env bash
set -e

echo "[1/7] Stop current broken processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/7] Restore mission_btcpp_node.cpp from backup before bad replacement..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
CPP_DIR = CPP.parent

def read(p):
    try:
        return p.read_text(errors="ignore")
    except Exception:
        return ""

def backup(p: Path, tag: str):
    if not p.exists():
        return
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def count_class(s, name):
    return len(re.findall(r"class\s+" + re.escape(name) + r"\s*:", s))

# 先备份当前坏掉的 C++
backup(CPP, "before_fix_duplicate_waitnavsettle")

# 优先找刚才替换前自动生成的备份
candidates = []
for p in CPP_DIR.glob("mission_btcpp_node.cpp.bak_replace_waitnavdone_*"):
    s = read(p)

    score = 0
    if "class WaitNavDoneBeforeOak" in s:
        score += 100
    if "registerNodeType<WaitNavDoneBeforeOak" in s:
        score += 100
    if "class WaitNavSettle" in s:
        score += 50
    if "class RequestOakPackageSearchAlign" in s:
        score += 50
    if "shelf_front_yaw" in s:
        score += 30
    if count_class(s, "WaitNavSettle") == 1:
        score += 80
    if count_class(s, "WaitNavSettle") > 1:
        score -= 200

    if score >= 200:
        candidates.append((score, p.stat().st_mtime, p))

if candidates:
    candidates.sort(reverse=True)
    score, mtime, best = candidates[0]
    print(f"[SELECT] restore cpp from {best}, score={score}")
    shutil.copy2(best, CPP)
    print(f"[OK] restored cpp: {CPP}")
else:
    print("[WARN] no clean bak_replace_waitnavdone backup found, repairing current cpp directly")

    s = read(CPP)

    # 如果当前 C++ 有两个 WaitNavSettle 类，把第二个改回 WaitNavDoneBeforeOak
    matches = list(re.finditer(r"class\s+WaitNavSettle\s*:", s))
    if len(matches) >= 2:
        second = matches[1].start()
        tail = s[second:]

        # 只处理第二个 class 到下一个 class 之前这一段
        next_class = tail.find("\nclass ", 10)
        if next_class < 0:
            next_class = len(tail)

        block = tail[:next_class]
        block2 = block.replace("class WaitNavSettle", "class WaitNavDoneBeforeOak", 1)
        block2 = block2.replace("WaitNavSettle(const std::string&n", "WaitNavDoneBeforeOak(const std::string&n", 1)
        block2 = block2.replace('ctx->set_state("WaitNavSettle")', 'ctx->set_state("WaitNavDoneBeforeOak")')

        s = s[:second] + block2 + tail[next_class:]

    # 去掉重复注册 WaitNavSettle
    dup = 'factory_.registerNodeType<WaitNavSettle>("WaitNavSettle");factory_.registerNodeType<WaitNavSettle>("WaitNavSettle");'
    while dup in s:
        s = s.replace(dup, 'factory_.registerNodeType<WaitNavSettle>("WaitNavSettle");')

    CPP.write_text(s)
    print("[OK] repaired current cpp directly")

# 最终检查
s = read(CPP)

print("[CHECK] class WaitNavSettle count =", count_class(s, "WaitNavSettle"))
print("[CHECK] class WaitNavDoneBeforeOak count =", count_class(s, "WaitNavDoneBeforeOak"))

if count_class(s, "WaitNavSettle") != 1:
    raise SystemExit("[ERROR] WaitNavSettle class count is not 1")

if "class RequestOakPackageSearchAlign" not in s:
    raise SystemExit("[ERROR] RequestOakPackageSearchAlign missing")

print("[OK] C++ structure fixed")
PY

echo "[3/7] Replace WaitNavDoneBeforeOak with WaitNavSettle in XML/YAML only, not C++..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation/robot_logistics_task"),
]

TARGET = "WaitNavDoneBeforeOak"
REPLACEMENT = "WaitNavSettle"

SUFFIXES = {".xml", ".yaml", ".yml"}

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_xml_waitnavsettle_only_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

changed = []

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if ".bak" in p.name:
            continue
        if p.suffix not in SUFFIXES:
            continue

        s = p.read_text(errors="ignore")
        if TARGET not in s:
            continue

        ns = s.replace(TARGET, REPLACEMENT)
        backup(p)
        p.write_text(ns)
        changed.append(p)
        print(f"[OK] patched XML/YAML: {p}")

print("[DONE] XML/YAML changed count =", len(changed))
PY

echo "[4/7] Clean old build/install for these packages..."
cd /host-home
rm -rf build/robot_logistics_task install/robot_logistics_task
rm -rf build/robot2_bringup install/robot2_bringup

echo "[5/7] Rebuild..."
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_logistics_task robot2_bringup --symlink-install

echo "[6/7] Source install..."
source /host-home/install/setup.bash

echo "[7/7] Final checks..."

echo ""
echo "[CHECK] C++ class count:"
grep -n "class WaitNavSettle\|class WaitNavDoneBeforeOak\|class RequestOakPackageSearchAlign" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] active XML/YAML should not use WaitNavDoneBeforeOak:"
grep -R -n "WaitNavDoneBeforeOak" \
/host-home/src/Navigation/robot_logistics_task/behavior_tree \
/host-home/src/Navigation/robot_logistics_task/config \
/host-home/install/robot_logistics_task/share/robot_logistics_task 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] active XML/YAML should use WaitNavSettle:"
grep -R -n "WaitNavSettle\|RequestOakPackageSearchAlign" \
/host-home/src/Navigation/robot_logistics_task/behavior_tree \
/host-home/src/Navigation/robot_logistics_task/config \
/host-home/install/robot_logistics_task/share/robot_logistics_task 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] binary contains WaitNavSettle and RequestOakPackageSearchAlign:"
strings /host-home/install/robot_logistics_task/lib/robot_logistics_task/mission_btcpp_node | grep -E "WaitNavSettle|RequestOakPackageSearchAlign" || true

echo ""
echo "[DONE] Fixed duplicate WaitNavSettle and rebuilt."
echo "Now restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
