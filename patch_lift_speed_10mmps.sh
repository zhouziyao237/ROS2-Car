#!/usr/bin/env bash
set -euo pipefail

cd /host-home

python3 - <<'PY'
from pathlib import Path
import re
import time

files = [
    Path("/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml"),
    Path("/host-home/install/oak_guided_pickup/share/oak_guided_pickup/config/oak_guided_pickup.yaml"),
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
    bak = p.with_suffix(p.suffix + ".bak_lift_speed_10mmps_" + stamp)
    bak.write_text(s)

    # 核心：升降 PID 最大速度，单位 mm/s。
    s = set_param(s, "max_lift_speed_mmps", "10.0")

    # 如果配置里有下降单独速度，也一起设成 10mm/s。
    possible_down_speed_keys = [
        "lift_down_speed_mmps",
        "lift_down_max_speed_mmps",
        "max_lift_down_speed_mmps",
        "startup_max_lift_speed_mmps",
        "startup_zero_max_lift_speed_mmps",
    ]

    for key in possible_down_speed_keys:
        if re.search(rf"(^\s*){re.escape(key)}\s*:", s, flags=re.M):
            s = set_param(s, key, "10.0")

    p.write_text(s)
    print("[patched]", p)
    print("[backup]", bak)

print("[done] lift speed set to 10.0 mm/s")
PY

echo "========== check oak_guided_pickup speed params =========="
grep -R "lift.*speed.*mmps\|speed.*lift.*mmps\|max_lift_speed_mmps" -n \
/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml \
/host-home/install/oak_guided_pickup/share/oak_guided_pickup/config/oak_guided_pickup.yaml || true
