#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time
import shutil
from pathlib import Path

MAP = Path("/host-home/maps/semantic_library.json")
APPROACH_DISTANCE = 0.75

if not MAP.exists():
    raise SystemExit(f"[ERROR] not found: {MAP}")

bak = MAP.with_suffix(MAP.suffix + f".bak_recompute_approach_{time.strftime('%Y%m%d_%H%M%S')}")
shutil.copy2(MAP, bak)
print(f"[BACKUP] {MAP} -> {bak}")

data = json.loads(MAP.read_text(encoding="utf-8"))

shelves = data.get("shelves", data)
count = 0

for shelf_id, shelf in shelves.items():
    if not isinstance(shelf, dict):
        continue

    cur = shelf.get("current_map_pose") or shelf.get("map_pose") or {}
    if not isinstance(cur, dict):
        continue

    if not cur.get("valid", True):
        continue

    try:
        cx = float(cur["x"])
        cy = float(cur["y"])
        yaw = float(cur.get("yaw", cur.get("shelf_front_yaw", 0.0)))
    except Exception:
        continue

    # 约定：current_map_pose.yaw / shelf_front_yaw 是货架正面朝外方向。
    nx = math.cos(yaw)
    ny = math.sin(yaw)

    ax = cx + APPROACH_DISTANCE * nx
    ay = cy + APPROACH_DISTANCE * ny

    # 车到 approach_pose 后，车头应该朝向货架中心。
    app_yaw = math.atan2(cy - ay, cx - ax)

    shelf["approach_pose"] = {
        "valid": True,
        "frame": "map",
        "x": ax,
        "y": ay,
        "yaw": app_yaw,
        "distance": APPROACH_DISTANCE,
        "source": "recomputed_from_current_map_pose"
    }

    count += 1
    print(f"[OK] {shelf_id}: center=({cx:.3f},{cy:.3f},{yaw:.3f}) -> approach=({ax:.3f},{ay:.3f},{app_yaw:.3f})")

MAP.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
print(f"[DONE] recomputed approach_pose for {count} shelves, distance={APPROACH_DISTANCE} m")
