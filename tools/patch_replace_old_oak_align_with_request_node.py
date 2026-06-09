#!/usr/bin/env python3
from pathlib import Path
import time
import shutil

STAMP = time.strftime("%Y%m%d_%H%M%S")

XML_DIRS = [
    Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree"),
    Path("/host-home/src/robot_logistics_task/behavior_tree"),
]

xml_files = []
for d in XML_DIRS:
    if d.exists():
        xml_files.extend(sorted(d.glob("*.xml")))

if not xml_files:
    raise SystemExit("[ERROR] cannot find behavior_tree XML files")

print("[INFO] found XML files:")
for p in xml_files:
    print("  ", p)

for p in xml_files:
    s = p.read_text(errors="ignore")
    original = s

    if "AlignOakToTargetShelfFront" not in s:
        print(f"[SKIP] no old AlignOakToTargetShelfFront in {p}")
        continue

    bak = p.with_suffix(p.suffix + f".bak_replace_old_oak_align_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BACKUP] {p} -> {bak}")

    # 1. Replace simple self-closing node.
    s = s.replace(
        "<AlignOakToTargetShelfFront/>",
        (
            "<!-- 使用已经调通的 oak_package_search_node 做 OAK 自动对正；"
            "不再使用旧的 oak_camera_link TF 对正节点。 -->\n"
            "      <RequestOakPackageSearchAlign/>"
        ),
    )

    # 2. Replace possible open-close form.
    s = s.replace(
        "<AlignOakToTargetShelfFront></AlignOakToTargetShelfFront>",
        (
            "<!-- 使用已经调通的 oak_package_search_node 做 OAK 自动对正；"
            "不再使用旧的 oak_camera_link TF 对正节点。 -->\n"
            "      <RequestOakPackageSearchAlign/>"
        ),
    )

    if s != original:
        p.write_text(s)
        print(f"[OK] replaced old OAK align node in {p}")
    else:
        print(f"[WARN] found name but replacement pattern did not match: {p}")

print("[DONE] XML patch completed")
