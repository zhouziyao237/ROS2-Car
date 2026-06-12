#!/usr/bin/env bash
set -e

echo "[1/6] Stop current robot task processes..."
pkill -f "yaml_bringup.launch.py|mission_btcpp_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|cmd_vel_mode_mux.py|oak_package_search_node|rm_serial_driver_node" || true
sleep 2

echo "[2/6] Restore good mission_btcpp_node.cpp from backups..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
TASK_ROOT = Path("/host-home/src/Navigation/robot_logistics_task")

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

candidates = []

for p in TASK_ROOT.rglob("mission_btcpp_node.cpp*"):
    if not p.is_file():
        continue

    s = read(p)

    score = 0

    # 必须有这些，才算原先完整任务版本
    if "class WaitNavDoneBeforeOak" in s:
        score += 100
    if "registerNodeType<WaitNavDoneBeforeOak" in s:
        score += 100
    if "class RequestOakPackageSearchAlign" in s:
        score += 50
    if "class ScanTargetPackageOnShelf" in s:
        score += 50
    if "class VerifyShelfQR" in s:
        score += 30
    if "class SendPlatformTarget" in s:
        score += 20

    # 过滤明显坏版本
    if "Error at line 9" in s:
        score -= 1000

    if score >= 200:
        candidates.append((score, p.stat().st_mtime, p))

if not candidates:
    print("[ERROR] 找不到包含 WaitNavDoneBeforeOak 的可用 C++ 备份。")
    print("[INFO] 你可以先执行下面命令看看还有哪些备份：")
    print("find /host-home/src/Navigation/robot_logistics_task -name 'mission_btcpp_node.cpp*' -type f")
    raise SystemExit(2)

candidates.sort(reverse=True)
score, mtime, best = candidates[0]

print(f"[SELECT] score={score} file={best}")

backup(CPP, "before_restore_good_cpp")
shutil.copy2(best, CPP)
print(f"[OK] restored C++ -> {CPP}")

# 检查恢复结果
s = read(CPP)
needles = [
    "class WaitNavDoneBeforeOak",
    "registerNodeType<WaitNavDoneBeforeOak",
    "class RequestOakPackageSearchAlign",
    "class ScanTargetPackageOnShelf",
]
for n in needles:
    if n not in s:
        print(f"[ERROR] restored C++ still missing: {n}")
        raise SystemExit(3)

print("[OK] C++ restored with WaitNavDoneBeforeOak and OAK align nodes.")
PY

echo "[3/6] Restore mission_task.yaml from best backup if available..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

YAML = Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml")
CONFIG_ROOT = Path("/host-home/src/Navigation/robot_logistics_task/config")

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

candidates = []

for p in CONFIG_ROOT.glob("mission_task.yaml*"):
    if not p.is_file():
        continue

    s = read(p)
    score = 0

    if "package_search_request_topic" in s:
        score += 30
    if "package_search_status_topic" in s:
        score += 30
    if "oak_align_max_wz" in s:
        score += 30
    if "package_scan_range_deg" in s:
        score += 20
    if "bt_xml" in s:
        score += 20
    if "mission_btcpp_node" in s:
        score += 20

    if score >= 60:
        candidates.append((score, p.stat().st_mtime, p))

if candidates:
    candidates.sort(reverse=True)
    score, mtime, best = candidates[0]
    print(f"[SELECT] score={score} yaml={best}")
    backup(YAML, "before_restore_good_yaml")
    shutil.copy2(best, YAML)
    print(f"[OK] restored YAML -> {YAML}")
else:
    print("[WARN] no better mission_task.yaml backup found, keep current YAML.")
PY

echo "[4/6] Ensure BT XML has WaitNavDoneBeforeOak before OAK align..."

python3 - <<'PY'
from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

BT_DIRS = [
    Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree"),
]

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_restore_waitnavdone_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")

def patch_xml_text(s: str):
    old = s

    # 只处理包含 OAK 对正节点的任务树
    if "RequestOakPackageSearchAlign" not in s:
        return s

    if "WaitNavDoneBeforeOak" in s:
        return s

    # 优先插在 WaitNavArrive 后面
    patterns = [
        r"(<\s*WaitNavArrive\s*/\s*>)",
        r"(<\s*WaitShelfNavigationArrive\s*/\s*>)",
    ]

    for pat in patterns:
        m = re.search(pat, s)
        if m:
            insert = m.group(1) + "\n        <WaitNavDoneBeforeOak/>"
            s = s[:m.start()] + insert + s[m.end():]
            return s

    return old

changed = 0

for d in BT_DIRS:
    if not d.exists():
        continue

    for p in d.rglob("*.xml"):
        s = p.read_text(errors="ignore")
        ns = patch_xml_text(s)

        if ns != s:
            backup(p)
            p.write_text(ns)
            print(f"[OK] patched BT XML: {p}")
            changed += 1

print(f"[DONE] patched BT XML count={changed}")

if changed == 0:
    print("[WARN] no XML patched. If your active BT XML already has WaitNavDoneBeforeOak, this is OK.")
PY

echo "[5/6] Build packages..."
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select robot_logistics_task robot2_bringup

echo "[6/6] Source install and verify..."
source /host-home/install/setup.bash

echo ""
echo "[CHECK] C++ must contain WaitNavDoneBeforeOak:"
grep -n "class WaitNavDoneBeforeOak\|registerNodeType<WaitNavDoneBeforeOak\|class RequestOakPackageSearchAlign\|class ScanTargetPackageOnShelf" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[CHECK] BT XML should contain WaitNavDoneBeforeOak before RequestOakPackageSearchAlign:"
grep -R -n "WaitNavDoneBeforeOak\|RequestOakPackageSearchAlign" \
/host-home/src/Navigation/robot_logistics_task/behavior_tree \
/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree 2>/dev/null || true

echo ""
echo "[DONE] Restored good OAK BT version."
echo "Now launch with:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
