from pathlib import Path
import re
import time

ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/src/Diverse"),
    Path("/host-home/src/target_detection"),
]

stamp = time.strftime("%Y%m%d_%H%M%S")

TARGET_PARAMS = {
    "output_frame_id": "camera_link",
    "convert_optical_to_base_link": True,
    "tag_to_shelf_yaw_offset_deg": 90.0,
}

SEMANTIC_PARAMS = {
    "front_qr_yaw_offset_deg": 0.0,
    "use_single_qr_normal_yaw": False,
}

def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_apriltag_yaw90_{stamp}")
    bak.write_text(p.read_text(errors="ignore"))
    print(f"[BACKUP] {p} -> {bak}")

def py_value(v):
    if isinstance(v, str):
        return repr(v)
    if isinstance(v, bool):
        return "True" if v else "False"
    return str(v)

def patch_param_in_dict_block(block: str, params: dict):
    changed = False

    for key, value in params.items():
        value_s = py_value(value)

        # Existing python dict item:
        # 'key': old
        pattern = rf"(['\"]{re.escape(key)}['\"]\s*:\s*)([^,\n}}]+)"
        if re.search(pattern, block):
            block2 = re.sub(pattern, rf"\g<1>{value_s}", block, count=1)
            if block2 != block:
                block = block2
                changed = True
        else:
            # Insert before closing } of first parameter dict.
            idx = block.rfind("}")
            if idx >= 0:
                insert = f"                '{key}': {value_s},\n"
                block = block[:idx] + insert + block[idx:]
                changed = True

    return block, changed

def patch_launch_file(p: Path):
    s = p.read_text(errors="ignore")
    old = s
    changed = False

    # Patch apriltag_shelf_bridge_node Node(...) block.
    if "apriltag_shelf_bridge_node" in s:
        pattern = re.compile(
            r"Node\((?:(?!\n\s*\),).)*?apriltag_shelf_bridge_node(?:(?!\n\s*\),).)*?\n\s*\)",
            re.S
        )

        def repl(m):
            nonlocal changed
            block = m.group(0)

            if "parameters=[" not in block:
                # Add parameter list before closing parenthesis.
                insert = (
                    "            parameters=[{\n"
                    "                'output_frame_id': 'camera_link',\n"
                    "                'convert_optical_to_base_link': True,\n"
                    "                'tag_to_shelf_yaw_offset_deg': 90.0,\n"
                    "            }],\n"
                )
                block = block.replace("\n        )", "\n" + insert + "        )")
                changed = True
                return block

            block2, c = patch_param_in_dict_block(block, TARGET_PARAMS)
            if c:
                changed = True
            return block2

        s = pattern.sub(repl, s)

    # Patch semantic_registrar_node Node(...) block to keep semantic layer neutral.
    if "semantic_registrar_node" in s:
        pattern = re.compile(
            r"Node\((?:(?!\n\s*\),).)*?semantic_registrar_node(?:(?!\n\s*\),).)*?\n\s*\)",
            re.S
        )

        def repl2(m):
            nonlocal changed
            block = m.group(0)
            if "parameters=[" not in block:
                return block
            block2, c = patch_param_in_dict_block(block, SEMANTIC_PARAMS)
            if c:
                changed = True
            return block2

        s = pattern.sub(repl2, s)

    if s != old:
        backup(p)
        p.write_text(s)
        print(f"[OK] patched launch: {p}")
        return True

    return False

def patch_yaml_file(p: Path):
    s = p.read_text(errors="ignore")
    old = s

    # Only patch files that mention apriltag_shelf_bridge_node or semantic_registrar_node.
    if "apriltag_shelf_bridge_node" not in s and "semantic_registrar_node" not in s:
        return False

    # YAML style replacement.
    repls = {
        "tag_to_shelf_yaw_offset_deg": "90.0",
        "output_frame_id": "camera_link",
        "convert_optical_to_base_link": "true",
        "front_qr_yaw_offset_deg": "0.0",
        "use_single_qr_normal_yaw": "false",
    }

    for key, value in repls.items():
        pattern = rf"(^\s*{re.escape(key)}\s*:\s*)(.+)$"
        if re.search(pattern, s, flags=re.M):
            s = re.sub(pattern, rf"\g<1>{value}", s, flags=re.M)

    if s != old:
        backup(p)
        p.write_text(s)
        print(f"[OK] patched yaml: {p}")
        return True

    return False

changed_files = []

for root in ROOTS:
    if not root.exists():
        continue
    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if p.suffix == ".py" and p.name.endswith(".launch.py"):
            if patch_launch_file(p):
                changed_files.append(p)
        elif p.suffix in [".yaml", ".yml"]:
            if patch_yaml_file(p):
                changed_files.append(p)

print("")
print("============================================================")
print("[SUMMARY]")
print("============================================================")

if changed_files:
    print("[OK] changed files:")
    for p in changed_files:
        print(f"  - {p}")
else:
    print("[WARN] No launch/yaml file was patched.")
    print("This means apriltag_shelf_bridge_node may be launched from another package or command.")
    print("Use this search command:")
    print("grep -R \"apriltag_shelf_bridge_node\" -n /host-home/src")

print("")
print("[EXPECTED FINAL PARAMS]")
print("/apriltag_shelf_bridge_node:")
print("  output_frame_id: camera_link")
print("  convert_optical_to_base_link: true")
print("  tag_to_shelf_yaw_offset_deg: 90.0")
print("/semantic_registrar_node:")
print("  front_qr_yaw_offset_deg: 0.0")
print("  use_single_qr_normal_yaw: false")
