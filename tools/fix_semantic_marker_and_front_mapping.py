from pathlib import Path
import time
import ast
import re
import shutil

REG = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py")
MARKER = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_marker_publisher_node.py")

stamp = time.strftime("%Y%m%d_%H%M%S")

def backup(p: Path):
    if p.exists():
        bak = p.with_suffix(p.suffix + f".bak_fix_marker_front_mapping_{stamp}")
        bak.write_text(p.read_text(errors="ignore"))
        print(f"[BACKUP] {p} -> {bak}")

def py_ok(text: str) -> bool:
    try:
        ast.parse(text)
        return True
    except Exception:
        return False

print("============================================================")
print("[STEP 1] Restore semantic_marker_publisher_node.py if broken")
print("============================================================")

if not MARKER.exists():
    raise SystemExit(f"[ERROR] not found: {MARKER}")

backup(MARKER)
marker_s = MARKER.read_text(errors="ignore")

if not py_ok(marker_s):
    print("[WARN] semantic_marker_publisher_node.py is syntactically broken. Searching backups...")

    backups = sorted(
        MARKER.parent.glob(MARKER.name + ".bak*"),
        key=lambda p: p.stat().st_mtime,
        reverse=True
    )

    restored = False
    for b in backups:
        bs = b.read_text(errors="ignore")
        if py_ok(bs) and "/semantic/shelf_markers" in bs and "body.scale.x" in bs:
            shutil.copyfile(b, MARKER)
            marker_s = MARKER.read_text(errors="ignore")
            print(f"[OK] Restored marker file from backup: {b}")
            restored = True
            break

    if not restored:
        raise SystemExit(
            "[ERROR] Could not find a valid backup for semantic_marker_publisher_node.py. "
            "Run: ls -lt /host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_marker_publisher_node.py.bak*"
        )
else:
    print("[OK] marker file syntax is valid before patch")

print("")
print("============================================================")
print("[STEP 2] Fix marker width/depth axis model")
print("============================================================")

marker_s = MARKER.read_text(errors="ignore")
lines = marker_s.splitlines(keepends=True)

patched_scale = False
for i in range(len(lines) - 1):
    if "body.scale.x" in lines[i]:
        # find nearby body.scale.y
        for j in range(i + 1, min(i + 5, len(lines))):
            if "body.scale.y" in lines[j]:
                indent = re.match(r"^(\s*)", lines[i]).group(1)

                new_block = [
                    indent + "# current_map_pose.yaw is shelf front outward normal.\n",
                    indent + "# Marker X axis = shelf depth direction.\n",
                    indent + "# Marker Y axis = shelf front long-face width direction.\n",
                    indent + "body.scale.x = depth\n",
                    indent + "body.scale.y = width\n",
                ]

                lines = lines[:i] + new_block + lines[j+1:]
                patched_scale = True
                print("[OK] marker scale model set to: x=depth, y=width")
                break
        if patched_scale:
            break

if not patched_scale:
    raise SystemExit("[ERROR] Could not find body.scale.x/body.scale.y block in marker file.")

marker_s = "".join(lines)

if not py_ok(marker_s):
    raise SystemExit("[ERROR] marker file still has Python syntax error after scale patch.")

MARKER.write_text(marker_s)

print("")
print("============================================================")
print("[STEP 3] Reset front layer QR mapping offset to 0 deg")
print("============================================================")

if not REG.exists():
    raise SystemExit(f"[ERROR] not found: {REG}")

backup(REG)
reg_s = REG.read_text(errors="ignore")

# 如果之前加过 front_layer_qr_to_normal_offset_deg，就把默认值设回 0.0
if "front_layer_qr_to_normal_offset_deg" in reg_s:
    reg_s2 = re.sub(
        r"(declare_parameter\(\s*['\"]front_layer_qr_to_normal_offset_deg['\"]\s*,\s*)([-+]?\d+(\.\d+)?)(\s*\))",
        r"\g<1>0.0\g<4>",
        reg_s,
        count=1,
    )

    reg_s2 = re.sub(
        r"(front_layer_qr_to_normal_offset_deg['\"]\s*:\s*)([-+]?\d+(\.\d+)?)",
        r"\g<1>0.0",
        reg_s2,
        count=1,
    )

    if reg_s2 != reg_s:
        reg_s = reg_s2
        print("[OK] front_layer_qr_to_normal_offset_deg reset to 0.0")
    else:
        print("[WARN] parameter exists but default value pattern was not changed")
else:
    print("[OK] no extra front_layer_qr_to_normal_offset_deg found; original mapping remains")

# 如果之前把 return 改成 obs_yaw + front_offset + front_layer_to_normal_offset，
# 保留变量不影响，但返回逻辑改回只用 front_offset。
old_return = """        if self.is_front_observation(obs_type, face):
            # Physical rule:
            #   layer_qr + face=front is a row/layer label on the long front face.
            # Internal rule used below:
            #   front_yaw is shelf front outward normal.
            # Therefore convert front-layer label yaw to front normal yaw here.
            return self.normalize_angle(obs_yaw + front_offset + front_layer_to_normal_offset)
"""

new_return = """        if self.is_front_observation(obs_type, face):
            # Front layer QR is mounted on the shelf front face.
            # The bridge already reports obs_yaw as the shelf front outward normal.
            # Do not rotate it by +/-90 here, otherwise front becomes side.
            return self.normalize_angle(obs_yaw + front_offset)
"""

if old_return in reg_s:
    reg_s = reg_s.replace(old_return, new_return, 1)
    print("[OK] front observation mapping restored to obs_yaw + front_offset")
else:
    # 更宽松地修一行
    reg_s2 = reg_s.replace(
        "return self.normalize_angle(obs_yaw + front_offset + front_layer_to_normal_offset)",
        "return self.normalize_angle(obs_yaw + front_offset)"
    )
    if reg_s2 != reg_s:
        reg_s = reg_s2
        print("[OK] front observation return line restored")
    else:
        print("[SKIP] front observation mapping already looks restored")

if not py_ok(reg_s):
    raise SystemExit("[ERROR] semantic_registrar_node.py syntax error after patch.")

REG.write_text(reg_s)

print("")
print("============================================================")
print("[STEP 4] Final syntax check")
print("============================================================")

for p in [MARKER, REG]:
    s = p.read_text(errors="ignore")
    ast.parse(s)
    print(f"[OK] Python syntax valid: {p}")

print("")
print("[DONE]")
print("Now rebuild robot_logistics_semantic and restart semantic nodes.")
