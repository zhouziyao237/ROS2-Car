#!/usr/bin/env bash
set -e

echo "[1/5] Stop current task processes..."
pkill -f "mission_btcpp_node|yaml_bringup.launch.py" || true
sleep 1

echo "[2/5] Patch all BT/XML/YAML files that still contain WaitNavDoneBeforeOak..."

python3 - <<'PY'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src"),
    Path("/host-home/install"),
]

TEXT_SUFFIX = {
    ".yaml", ".yml", ".xml", ".py", ".launch", ".txt", ".cpp", ".hpp", ".h"
}

TARGET = "WaitNavDoneBeforeOak"


def is_backup(p: Path) -> bool:
    name = p.name
    return ".bak" in name or name.endswith("~") or name.endswith(".old")


def is_text_candidate(p: Path) -> bool:
    if not p.is_file():
        return False
    if is_backup(p):
        return False
    if p.suffix in TEXT_SUFFIX:
        return True
    if "launch" in p.name or "mission" in p.name or "bt" in p.name:
        return True
    return False


def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_remove_waitnavdone_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def patch_text(s: str) -> str:
    old = s

    # 删除常见自闭合 XML 节点
    s = re.sub(r"\n?[ \t]*<\s*WaitNavDoneBeforeOak\s*/\s*>[ \t]*", "\n", s)

    # 删除成对 XML 节点
    s = re.sub(
        r"\n?[ \t]*<\s*WaitNavDoneBeforeOak\s*>\s*</\s*WaitNavDoneBeforeOak\s*>[ \t]*",
        "\n",
        s,
        flags=re.S,
    )

    # 如果 YAML/XML 字符串里有同一行紧挨着的情况，再清一次
    s = s.replace("<WaitNavDoneBeforeOak/>", "")
    s = s.replace("<WaitNavDoneBeforeOak />", "")
    s = s.replace("<WaitNavDoneBeforeOak></WaitNavDoneBeforeOak>", "")

    # 不删除 C++ 注册行，因为当前问题是 XML 还引用它；
    # 如果 C++ 里面有注册残留但没有类，编译会报错，这里额外注释掉注册残留。
    s = re.sub(
        r"factory_\.registerNodeType\s*<\s*WaitNavDoneBeforeOak\s*>\s*\(\s*\"WaitNavDoneBeforeOak\"\s*\)\s*;",
        "/* removed stale WaitNavDoneBeforeOak registration */",
        s,
    )

    return s


changed_files = []

for root in ROOTS:
    if not root.exists():
        continue

    for p in root.rglob("*"):
        if not is_text_candidate(p):
            continue

        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue

        if TARGET not in s:
            continue

        new_s = patch_text(s)

        if new_s != s:
            backup(p)
            p.write_text(new_s)
            changed_files.append(p)
            print(f"[OK] patched: {p}")
        else:
            print(f"[WARN] contains target but no known pattern patched: {p}")

print("")
print("[DONE] patched files:")
for p in changed_files:
    print(" ", p)

print("")
print("[CHECK] Remaining non-backup occurrences:")
remaining = []
for root in ROOTS:
    if not root.exists():
        continue
    for p in root.rglob("*"):
        if not is_text_candidate(p):
            continue
        try:
            s = p.read_text(errors="ignore")
        except Exception:
            continue
        if TARGET in s:
            remaining.append(p)

if remaining:
    for p in remaining:
        print("[LEFT]", p)
    raise SystemExit("Still has WaitNavDoneBeforeOak in non-backup files. Please send the [LEFT] list.")
else:
    print("[OK] no WaitNavDoneBeforeOak remains in active text files.")
PY

echo "[3/5] Build robot_logistics_task and robot2_bringup..."
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select robot_logistics_task robot2_bringup

echo "[4/5] Source new install..."
source /host-home/install/setup.bash

echo "[5/5] Final check..."
echo ""
echo "[CHECK] WaitNavDoneBeforeOak should be empty below:"
grep -R -n "WaitNavDoneBeforeOak" /host-home/src /host-home/install 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[CHECK] Asymmetric scan params:"
grep -R -n "package_scan_left_deg\|package_scan_right_deg\|package_scan_wz_degps\|package_scan_step_deg" \
/host-home/src/Navigation/robot_logistics_task \
/host-home/install/robot_logistics_task 2>/dev/null | grep -v ".bak" || true

echo ""
echo "[DONE] Fix complete. Now restart bringup manually:"
echo "cd /host-home"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task"
