from pathlib import Path
import re
import time

ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/src/Diverse"),
    Path("/host-home/src/target_detection"),
]

stamp = time.strftime("%Y%m%d_%H%M%S")

# 你截图里当前绿色货架相对红框差 90 度。
# 先用 -90。如果之后发现转反了，把这里改成 90.0 再执行一次。
YAW_OFFSET_DEG = -90.0

KEYS = [
    "front_yaw_offset_deg",
    "shelf_yaw_offset_deg",
    "shelf_front_yaw_offset_deg",
    "layer_qr_yaw_offset_deg",
    "front_layer_qr_yaw_offset_deg",
    "footprint_yaw_offset_deg",
    "marker_yaw_offset_deg",
    "semantic_shelf_yaw_offset_deg",
]

SEARCH_HINTS = [
    "front_layer_qr",
    "layer_qr",
    "shelf_yaw",
    "front_yaw",
    "yaw_offset",
    "footprint",
    "semantic",
    "shelf",
]

def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_shelf_yaw_offset_{stamp}")
    bak.write_text(path.read_text(errors="ignore"))
    print(f"[BACKUP] {path} -> {bak}")

def is_candidate(path: Path) -> bool:
    if path.suffix not in [".py", ".yaml", ".yml", ".launch.py"]:
        return False

    try:
        s = path.read_text(errors="ignore")
    except Exception:
        return False

    lower = s.lower()
    return any(k.lower() in lower for k in SEARCH_HINTS)

def patch_yaml_style(s: str):
    changed = False

    for key in KEYS:
        # YAML:
        # front_yaw_offset_deg: 0.0
        pattern = rf"(^\s*{re.escape(key)}\s*:\s*)([-+]?\d+(\.\d+)?)"
        repl = rf"\g<1>{YAW_OFFSET_DEG}"
        s2 = re.sub(pattern, repl, s, flags=re.M)
        if s2 != s:
            s = s2
            changed = True

    return s, changed

def patch_python_declare_parameter(s: str):
    changed = False

    for key in KEYS:
        # self.declare_parameter("front_yaw_offset_deg", 0.0)
        pattern = rf"(declare_parameter\(\s*['\"]{re.escape(key)}['\"]\s*,\s*)([-+]?\d+(\.\d+)?)(\s*\))"
        repl = rf"\g<1>{YAW_OFFSET_DEG}\g<4>"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

        # ("front_yaw_offset_deg", 0.0)
        pattern = rf"(\(\s*['\"]{re.escape(key)}['\"]\s*,\s*)([-+]?\d+(\.\d+)?)(\s*\))"
        repl = rf"\g<1>{YAW_OFFSET_DEG}\g<4>"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

        # "front_yaw_offset_deg": 0.0
        pattern = rf"(['\"]{re.escape(key)}['\"]\s*:\s*)([-+]?\d+(\.\d+)?)"
        repl = rf"\g<1>{YAW_OFFSET_DEG}"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

    return s, changed

def create_override_yaml():
    # 创建一个明确的 override 配置，后续如果 launch 有加载 config，可以手动 include。
    config_dir = Path("/host-home/src/Navigation/robot_logistics_semantic/config")
    if not config_dir.exists():
        config_dir.mkdir(parents=True, exist_ok=True)

    p = config_dir / "semantic_shelf_front_geometry_override.yaml"

    content = f"""# Semantic shelf front geometry override
# L / layer_qr is mounted on the FRONT face of the shelf.
# The shelf footprint should be placed like the red rectangle in RViz.
# If the shelf rotates to the opposite side, change -90.0 to 90.0.

semantic_registrar_node:
  ros__parameters:
    front_yaw_offset_deg: {YAW_OFFSET_DEG}
    shelf_yaw_offset_deg: {YAW_OFFSET_DEG}
    shelf_front_yaw_offset_deg: {YAW_OFFSET_DEG}
    layer_qr_yaw_offset_deg: {YAW_OFFSET_DEG}
    front_layer_qr_yaw_offset_deg: {YAW_OFFSET_DEG}
    footprint_yaw_offset_deg: {YAW_OFFSET_DEG}
    marker_yaw_offset_deg: {YAW_OFFSET_DEG}
    semantic_shelf_yaw_offset_deg: {YAW_OFFSET_DEG}

apriltag_shelf_bridge_node:
  ros__parameters:
    front_yaw_offset_deg: {YAW_OFFSET_DEG}
    shelf_yaw_offset_deg: {YAW_OFFSET_DEG}
    shelf_front_yaw_offset_deg: {YAW_OFFSET_DEG}
    layer_qr_yaw_offset_deg: {YAW_OFFSET_DEG}
    front_layer_qr_yaw_offset_deg: {YAW_OFFSET_DEG}
    footprint_yaw_offset_deg: {YAW_OFFSET_DEG}
    marker_yaw_offset_deg: {YAW_OFFSET_DEG}
    semantic_shelf_yaw_offset_deg: {YAW_OFFSET_DEG}
"""

    if p.exists():
        backup(p)

    p.write_text(content)
    print(f"[WRITE] override yaml: {p}")

print("============================================================")
print("[STEP 1] Create override yaml")
print("============================================================")
create_override_yaml()

print("")
print("============================================================")
print("[STEP 2] Patch existing semantic shelf yaw parameters")
print("============================================================")

changed_files = []
candidate_files = []

for root in ROOTS:
    if not root.exists():
        continue
    for path in root.rglob("*"):
        if path.is_file() and is_candidate(path):
            candidate_files.append(path)

print(f"[INFO] candidate files: {len(candidate_files)}")

for path in candidate_files:
    s = path.read_text(errors="ignore")
    old = s

    s, c1 = patch_yaml_style(s)
    s, c2 = patch_python_declare_parameter(s)

    if s != old:
        backup(path)
        path.write_text(s)
        changed_files.append(path)
        print(f"[OK] patched {path}")

print("")
print("============================================================")
print("[STEP 3] Candidate files containing shelf/front/yaw keywords")
print("============================================================")

for path in candidate_files[:80]:
    print(f"  - {path}")

print("")
print("============================================================")
print("[SUMMARY]")
print("============================================================")

if changed_files:
    print("[OK] Patched files:")
    for p in changed_files:
        print(f"  - {p}")
else:
    print("[WARN] No existing parameter was patched.")
    print("[WARN] This means your current code may not expose yaw offset as a parameter.")
    print("[WARN] Use the grep command below to locate the hard-coded yaw formula.")

print("")
print(f"[DONE] Requested yaw offset = {YAW_OFFSET_DEG} deg")
print("")
print("If RViz shelf direction is still rotated the wrong way, edit this script:")
print("  YAW_OFFSET_DEG = 90.0")
print("and run it again.")
