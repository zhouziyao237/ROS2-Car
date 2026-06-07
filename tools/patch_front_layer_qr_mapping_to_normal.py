from pathlib import Path
import re
import time

REG = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py")
MARKER = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_marker_publisher_node.py")

# 你的现象是货架整体差 90°。
# 先用 -90。如果结果变成反方向，再改成 90.0 后重新执行。
OFFSET_DEG = -90.0

stamp = time.strftime("%Y%m%d_%H%M%S")

def backup(p: Path):
    if p.exists():
        bak = p.with_suffix(p.suffix + f".bak_front_layer_qr_mapping_{stamp}")
        bak.write_text(p.read_text(errors="ignore"))
        print(f"[BACKUP] {p} -> {bak}")

if not REG.exists():
    raise SystemExit(f"[ERROR] not found: {REG}")

backup(REG)
s = REG.read_text(errors="ignore")

# ------------------------------------------------------------
# 1. 增加参数 front_layer_qr_to_normal_offset_deg
# ------------------------------------------------------------
if "front_layer_qr_to_normal_offset_deg" not in s:
    anchor = "        self.declare_parameter('front_qr_yaw_offset_deg'"
    idx = s.find(anchor)
    if idx < 0:
        raise SystemExit("[ERROR] cannot find front_qr_yaw_offset_deg declaration")

    line_end = s.find("\n", idx)
    insert = (
        "\n        # front layer QR / row label is mounted on the long front face.\n"
        "        # Its observed yaw may follow the label/long-face direction.\n"
        "        # Convert it to shelf front outward normal used internally.\n"
        f"        self.declare_parameter('front_layer_qr_to_normal_offset_deg', {OFFSET_DEG})"
    )
    s = s[:line_end] + insert + s[line_end:]
    print("[OK] added parameter front_layer_qr_to_normal_offset_deg")
else:
    # 如果已经存在，直接改默认值
    s2 = re.sub(
        r"(declare_parameter\(\s*['\"]front_layer_qr_to_normal_offset_deg['\"]\s*,\s*)([-+]?\d+(\.\d+)?)(\s*\))",
        rf"\g<1>{OFFSET_DEG}\g<4>",
        s,
        count=1,
    )
    if s2 != s:
        s = s2
        print("[OK] updated front_layer_qr_to_normal_offset_deg default")
    else:
        print("[SKIP] parameter already exists but default not matched")

# ------------------------------------------------------------
# 2. 修改 decide_front_yaw 里 front observation 的映射
# ------------------------------------------------------------
old = """        front_offset = math.radians(float(self.get_parameter('front_qr_yaw_offset_deg').value))
        right_offset = math.radians(float(self.get_parameter('right_side_qr_yaw_offset_deg').value))
        left_offset = math.radians(float(self.get_parameter('left_side_qr_yaw_offset_deg').value))
"""

new = """        front_offset = math.radians(float(self.get_parameter('front_qr_yaw_offset_deg').value))
        front_layer_to_normal_offset = math.radians(
            float(self.get_parameter('front_layer_qr_to_normal_offset_deg').value)
        )
        right_offset = math.radians(float(self.get_parameter('right_side_qr_yaw_offset_deg').value))
        left_offset = math.radians(float(self.get_parameter('left_side_qr_yaw_offset_deg').value))
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[OK] added front_layer_to_normal_offset in decide_front_yaw")
elif "front_layer_to_normal_offset" in s:
    print("[SKIP] front_layer_to_normal_offset already exists in decide_front_yaw")
else:
    raise SystemExit("[ERROR] cannot patch offset variable block")

old = """        if self.is_front_observation(obs_type, face):
            return self.normalize_angle(obs_yaw + front_offset)
"""

new = """        if self.is_front_observation(obs_type, face):
            # Physical rule:
            #   layer_qr + face=front is a row/layer label on the long front face.
            # Internal rule used below:
            #   front_yaw is shelf front outward normal.
            # Therefore convert front-layer label yaw to front normal yaw here.
            return self.normalize_angle(obs_yaw + front_offset + front_layer_to_normal_offset)
"""

if old in s:
    s = s.replace(old, new, 1)
    print("[OK] patched front/layer QR yaw mapping")
elif "front_layer_to_normal_offset" in s and "layer_qr + face=front" in s:
    print("[SKIP] front/layer QR yaw mapping already patched")
else:
    raise SystemExit("[ERROR] cannot patch front observation return block")

REG.write_text(s)

# ------------------------------------------------------------
# 3. 恢复 marker 显示逻辑：
# current_map_pose.yaw 仍然是 front outward normal，
# 所以 marker X 轴是 depth，Y 轴是 front long-face width。
# ------------------------------------------------------------
if MARKER.exists():
    backup(MARKER)
    m = MARKER.read_text(errors="ignore")

    # 不管之前是不是被改成 width/depth，都统一成 normal 模型。
    pattern = re.compile(
        r"(\s*#.*?yaw.*?\n)?(\s*)body\.scale\.x\s*=\s*(width|depth)\s*\n\s*body\.scale\.y\s*=\s*(width|depth)",
        re.S,
    )

    replacement = (
        "            # current_map_pose.yaw is shelf front outward normal.\n"
        "            # Marker X axis = shelf depth direction.\n"
        "            # Marker Y axis = shelf front long-face width direction.\n"
        "            body.scale.x = depth\n"
        "            body.scale.y = width"
    )

    if "body.scale.x" in m and "body.scale.y" in m:
        m2 = pattern.sub(replacement, m, count=1)
        if m2 != m:
            MARKER.write_text(m2)
            print("[OK] normalized marker width/depth display model")
        else:
            print("[WARN] marker scale block not changed; inspect manually if display still wrong")
    else:
        print("[WARN] marker file has no body.scale.x/body.scale.y")
else:
    print(f"[WARN] marker file not found: {MARKER}")

print("")
print("[DONE]")
print(f"front_layer_qr_to_normal_offset_deg = {OFFSET_DEG}")
print("")
print("If the shelf becomes 180-degree opposite or rotates the wrong way, change OFFSET_DEG to 90.0 and run again.")
