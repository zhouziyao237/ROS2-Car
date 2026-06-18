#!/usr/bin/env bash
set -euo pipefail

cd /host-home

python3 - <<'PY'
from pathlib import Path
import re
import time

files = [
    Path("/host-home/src/target_capture/oak_package_grabber/config/oak_package_grabber.yaml"),
    Path("/host-home/install/oak_package_grabber/share/oak_package_grabber/config/oak_package_grabber.yaml"),
]

stamp = time.strftime("%Y%m%d_%H%M%S")

def set_param(text: str, key: str, value: str) -> str:
    if re.search(rf"(^\s*){re.escape(key)}\s*:", text, flags=re.M):
        return re.sub(
            rf"(^\s*){re.escape(key)}\s*:.*$",
            rf"\1{key}: {value}",
            text,
            flags=re.M,
        )
    if "ros__parameters:" in text:
        return text.rstrip() + f"\n    {key}: {value}\n"
    return text.rstrip() + f"\n{key}: {value}\n"

for p in files:
    if not p.exists():
        print("[skip missing]", p)
        continue

    s = p.read_text()
    bak = p.with_suffix(p.suffix + ".bak_fix_grab_direction_" + stamp)
    bak.write_text(s)

    # 你的实测结果：
    # close_gripper 当前会物理开爪，说明 true 才是 OPEN。
    s = set_param(s, "grab_open_value", "true")
    s = set_param(s, "grab_close_value", "false")

    # 搜索阶段继续保持开爪。
    s = set_param(s, "hold_open_after_open_gripper", "true")
    s = set_param(s, "hold_open_max_sec", "60.0")
    s = set_param(s, "open_wait_sec", "3.0")

    p.write_text(s)
    print("[patched]", p)
    print("[backup]", bak)

print("[done] fixed grab direction: open=true, close=false")
PY

echo "[check]"
grep -n "grab_open_value\|grab_close_value\|hold_open_max_sec\|open_wait_sec" \
/host-home/src/target_capture/oak_package_grabber/config/oak_package_grabber.yaml
