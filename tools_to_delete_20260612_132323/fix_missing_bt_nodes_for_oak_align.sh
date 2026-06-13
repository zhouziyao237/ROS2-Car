#!/usr/bin/env bash
set -e

echo "[1/5] Stop current broken mission process..."
pkill -f "mission_btcpp_node|yaml_bringup.launch.py" || true
sleep 1

echo "[2/5] Patch mission_btcpp_node.cpp: add missing BT nodes / aliases..."

python3 - <<'PY'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")


def backup(p: Path, tag: str):
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def has_register(s: str, node_name: str) -> bool:
    pat = r'registerNodeType\s*<[^>]+>\s*\(\s*"' + re.escape(node_name) + r'"\s*\)'
    return re.search(pat, s) is not None


def insert_after_pattern(s: str, pattern: str, insert_text: str, label: str) -> str:
    m = re.search(pattern, s)
    if not m:
        raise RuntimeError(f"cannot find insert point: {label}")
    return s[:m.end()] + insert_text + s[m.end():]


if not CPP.exists():
    raise FileNotFoundError(CPP)

s = CPP.read_text(errors="ignore")
old = s

backup(CPP, "before_fix_missing_bt_nodes")

# ------------------------------------------------------------
# 1. 补 class WaitNavDoneBeforeOak
#    作用：Nav2 到点后给 1 秒缓冲，避免马上抢 OAK 对正。
# ------------------------------------------------------------
if "class WaitNavDoneBeforeOak" not in s:
    if "class TimedAction" not in s or "class WaitNavArrive" not in s:
        raise RuntimeError("cannot add WaitNavDoneBeforeOak: TimedAction or WaitNavArrive missing")

    cls = (
        '\n'
        'class WaitNavDoneBeforeOak:public TimedAction{'
        'public:'
        'WaitNavDoneBeforeOak(const std::string&n,const BT::NodeConfiguration&c)'
        ':TimedAction(n,c,"WaitNavDoneBeforeOak",1.0){}'
        'SIMPLE_PORTS'
        '};\n'
    )

    # 插在 WaitNavArrive 后面
    s = re.sub(
        r'(class\s+WaitNavArrive\s*:\s*public\s+TimedAction\s*\{.*?\};)',
        r'\1' + cls,
        s,
        count=1,
        flags=re.S
    )

    if "class WaitNavDoneBeforeOak" not in s:
        raise RuntimeError("failed to insert class WaitNavDoneBeforeOak")

    print("[OK] added class WaitNavDoneBeforeOak")
else:
    print("[NO CHANGE] class WaitNavDoneBeforeOak already exists")


# ------------------------------------------------------------
# 2. 可选补 class WaitAfterOakAlign
#    有些 XML 里会用这个节点；没有也不影响。
# ------------------------------------------------------------
active_text = ""
for root in [
    Path("/host-home/src/Navigation/robot_logistics_task"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task"),
]:
    if root.exists():
        for p in root.rglob("*"):
            if p.is_file() and p.suffix in [".xml", ".yaml", ".yml"]:
                try:
                    active_text += "\n" + p.read_text(errors="ignore")
                except Exception:
                    pass

if "WaitAfterOakAlign" in active_text and "class WaitAfterOakAlign" not in s:
    cls = (
        '\n'
        'class WaitAfterOakAlign:public TimedAction{'
        'public:'
        'WaitAfterOakAlign(const std::string&n,const BT::NodeConfiguration&c)'
        ':TimedAction(n,c,"WaitAfterOakAlign",0.5){}'
        'SIMPLE_PORTS'
        '};\n'
    )

    s = re.sub(
        r'(class\s+WaitNavDoneBeforeOak\s*:\s*public\s+TimedAction\s*\{.*?\};)',
        r'\1' + cls,
        s,
        count=1,
        flags=re.S
    )

    if "class WaitAfterOakAlign" not in s:
        raise RuntimeError("failed to insert class WaitAfterOakAlign")

    print("[OK] added class WaitAfterOakAlign")


# ------------------------------------------------------------
# 3. 注册 WaitNavDoneBeforeOak
# ------------------------------------------------------------
if not has_register(s, "WaitNavDoneBeforeOak"):
    insert = 'factory_.registerNodeType<WaitNavDoneBeforeOak>("WaitNavDoneBeforeOak");'

    pat = r'factory_\.registerNodeType\s*<\s*WaitNavArrive\s*>\s*\(\s*"WaitNavArrive"\s*\)\s*;'
    s = insert_after_pattern(s, pat, insert, "register WaitNavArrive")

    print("[OK] registered WaitNavDoneBeforeOak")
else:
    print("[NO CHANGE] WaitNavDoneBeforeOak already registered")


# ------------------------------------------------------------
# 4. 注册 WaitAfterOakAlign
# ------------------------------------------------------------
if "class WaitAfterOakAlign" in s and not has_register(s, "WaitAfterOakAlign"):
    insert = 'factory_.registerNodeType<WaitAfterOakAlign>("WaitAfterOakAlign");'

    pat = r'factory_\.registerNodeType\s*<\s*WaitNavDoneBeforeOak\s*>\s*\(\s*"WaitNavDoneBeforeOak"\s*\)\s*;'
    s = insert_after_pattern(s, pat, insert, "register WaitNavDoneBeforeOak")

    print("[OK] registered WaitAfterOakAlign")


# ------------------------------------------------------------
# 5. 修 RequestOakPackageSearchAlign
#    如果当前 C++ 有真正的 RequestOakPackageSearchAlign 类，就注册它；
#    如果没有，就把 XML 里的 RequestOakPackageSearchAlign 映射到 AlignOakToTargetShelfFront。
#    这样对正节点不会丢。
# ------------------------------------------------------------
if not has_register(s, "RequestOakPackageSearchAlign"):

    if "class RequestOakPackageSearchAlign" in s:
        reg_type = "RequestOakPackageSearchAlign"
        print("[INFO] using real class RequestOakPackageSearchAlign")
    elif "class AlignOakToTargetShelfFront" in s:
        reg_type = "AlignOakToTargetShelfFront"
        print("[INFO] RequestOakPackageSearchAlign mapped to AlignOakToTargetShelfFront")
    else:
        raise RuntimeError("no RequestOakPackageSearchAlign class and no AlignOakToTargetShelfFront class")

    insert = f'factory_.registerNodeType<{reg_type}>("RequestOakPackageSearchAlign");'

    if "AlignOakToTargetShelfFront" in s:
        pat = r'factory_\.registerNodeType\s*<\s*AlignOakToTargetShelfFront\s*>\s*\(\s*"AlignOakToTargetShelfFront"\s*\)\s*;'
        s = insert_after_pattern(s, pat, insert, "register AlignOakToTargetShelfFront")
    else:
        pat = r'factory_\.registerNodeType\s*<\s*ScanTargetPackageOnShelf\s*>\s*\(\s*"ScanTargetPackageOnShelf"\s*\)\s*;'
        s = insert_after_pattern(s, pat, insert, "register ScanTargetPackageOnShelf")

    print("[OK] registered RequestOakPackageSearchAlign")
else:
    print("[NO CHANGE] RequestOakPackageSearchAlign already registered")


# ------------------------------------------------------------
# 6. 最终写回
# ------------------------------------------------------------
if s == old:
    print("[NO CHANGE] cpp unchanged")
else:
    CPP.write_text(s)
    print(f"[OK] patched cpp: {CPP}")


# ------------------------------------------------------------
# 7. 基础检查
# ------------------------------------------------------------
checks = [
    "class WaitNavDoneBeforeOak",
    '"WaitNavDoneBeforeOak"',
    '"RequestOakPackageSearchAlign"',
]

for c in checks:
    if c not in s:
        raise RuntimeError(f"check failed, missing: {c}")

print("[DONE] C++ BT node compatibility fixed")
PY


echo "[3/5] Build packages..."
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select robot_logistics_task robot2_bringup

echo "[4/5] Source install..."
source /host-home/install/setup.bash

echo "[5/5] Check active C++ registrations..."
grep -n "WaitNavDoneBeforeOak\|RequestOakPackageSearchAlign\|AlignOakToTargetShelfFront" \
/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp || true

echo ""
echo "[DONE] Now restart:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
