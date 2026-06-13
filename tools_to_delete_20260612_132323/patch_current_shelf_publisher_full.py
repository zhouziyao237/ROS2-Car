#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

CANDIDATE_NODE_FILES = [
    Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py"),
    Path("/host-home/src/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py"),
]

BRINGUP_YAML = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")


def find_node_file() -> Path:
    for p in CANDIDATE_NODE_FILES:
        if p.exists():
            return p

    found = list(Path("/host-home/src").rglob("semantic_registrar_node.py"))
    if found:
        return found[0]

    raise SystemExit("[ERROR] Cannot find semantic_registrar_node.py under /host-home/src")


def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_current_shelf_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BACKUP] {p} -> {bak}")


def ensure_imports(s: str) -> str:
    if "import json" not in s:
        # Prefer inserting near other standard imports.
        if "import math" in s:
            s = s.replace("import math\n", "import math\nimport json\n", 1)
        else:
            s = "import json\n" + s
        print("[OK] added import json")

    if "from std_msgs.msg import String" not in s:
        # If there is an existing std_msgs import, extend it.
        m = re.search(r"^from std_msgs\.msg import ([^\n]+)$", s, flags=re.M)
        if m:
            names = [x.strip() for x in m.group(1).split(",")]
            if "String" not in names:
                names.append("String")
            new_line = "from std_msgs.msg import " + ", ".join(names)
            s = s[:m.start()] + new_line + s[m.end():]
            print("[OK] extended std_msgs.msg import with String")
        else:
            # Insert after other imports if possible.
            imports = list(re.finditer(r"^(import .+|from .+ import .+)$", s, flags=re.M))
            if imports:
                pos = imports[-1].end()
                s = s[:pos] + "\nfrom std_msgs.msg import String" + s[pos:]
            else:
                s = "from std_msgs.msg import String\n" + s
            print("[OK] added from std_msgs.msg import String")

    return s


def get_function_block(s: str, func_name: str):
    pattern = rf"\n    def {func_name}\(.*?\n(?=\n    def |\nclass |\Z)"
    m = re.search(pattern, s, flags=re.S)
    if not m:
        return None
    return m


def patch_init(s: str) -> str:
    m = get_function_block(s, "__init__")
    if not m:
        raise SystemExit("[ERROR] cannot find __init__() in semantic_registrar_node.py")

    block = m.group(0)
    new_block = block

    # Add parameters if missing.
    param_lines = []
    if "current_shelf_topic" not in new_block:
        param_lines.append("        self.declare_parameter('current_shelf_topic', '/semantic/current_shelf')")
    if "current_shelf_republish_period_sec" not in new_block:
        param_lines.append("        self.declare_parameter('current_shelf_republish_period_sec', 0.5)")

    if param_lines:
        insert_text = "\n".join(param_lines) + "\n"

        decls = list(re.finditer(r"^        self\.declare_parameter\(.*\)\s*$", new_block, flags=re.M))
        if decls:
            pos = decls[-1].end()
            new_block = new_block[:pos] + "\n" + insert_text + new_block[pos:]
        else:
            sup = re.search(r"super\(\).__init__\([^\n]*\)\n", new_block)
            if not sup:
                raise SystemExit("[ERROR] cannot find parameter insertion point in __init__")
            pos = sup.end()
            new_block = new_block[:pos] + insert_text + new_block[pos:]

        print("[OK] added current_shelf parameters")

    # Add cache + publisher + timer if missing.
    if "self.current_shelf_pub" not in new_block:
        insert_text = (
            "\n"
            "        # Publisher for the latest semantic shelf pose.\n"
            "        # This fills /semantic/current_shelf for BT / OSD / OAK package search.\n"
            "        self._last_current_shelf_msg = None\n"
            "        self.current_shelf_pub = self.create_publisher(\n"
            "            String,\n"
            "            str(self.get_parameter('current_shelf_topic').value),\n"
            "            10,\n"
            "        )\n"
            "        self.current_shelf_timer = self.create_timer(\n"
            "            max(0.1, float(self.get_parameter('current_shelf_republish_period_sec').value)),\n"
            "            self.republish_current_shelf,\n"
            "        )\n"
        )

        # Prefer after last create_publisher in __init__.
        pubs = list(re.finditer(r"^        self\.\w+\s*=\s*self\.create_publisher\(.*?\)\s*$", new_block, flags=re.M | re.S))
        if pubs:
            pos = pubs[-1].end()
            new_block = new_block[:pos] + insert_text + new_block[pos:]
        else:
            # Prefer after last declare_parameter if no publisher exists.
            decls = list(re.finditer(r"^        self\.declare_parameter\(.*\)\s*$", new_block, flags=re.M))
            if decls:
                pos = decls[-1].end()
                new_block = new_block[:pos] + insert_text + new_block[pos:]
            else:
                sup = re.search(r"super\(\).__init__\([^\n]*\)\n", new_block)
                if not sup:
                    raise SystemExit("[ERROR] cannot find publisher insertion point in __init__")
                pos = sup.end()
                new_block = new_block[:pos] + insert_text + new_block[pos:]

        print("[OK] added current_shelf publisher + republish timer")
    else:
        print("[SKIP] current_shelf publisher already exists")

    return s[:m.start()] + new_block + s[m.end():]


def add_helper_methods(s: str) -> str:
    if "def publish_current_shelf(" in s and "def republish_current_shelf(" in s:
        print("[SKIP] helper methods already exist")
        return s

    helper = r'''
    def _extract_shelf_id_for_current_shelf(self, shelf):
        try:
            return str(
                shelf.get('shelf_id')
                or shelf.get('id')
                or shelf.get('name')
                or shelf.get('label')
                or ''
            )
        except Exception:
            return ''

    def publish_current_shelf(self, shelf, stamp=None, source='semantic_registrar'):
        """
        Publish the latest recognized shelf pose to /semantic/current_shelf.

        Expected shelf structure already exists in semantic_registrar:
          shelf['current_map_pose'] = {'x': ..., 'y': ..., 'yaw': ..., 'valid': True}
          shelf['approach_pose']    = {'x': ..., 'y': ..., 'yaw': ..., 'valid': True}
        """
        try:
            if shelf is None:
                return

            pose = shelf.get('current_map_pose', {})
            approach = shelf.get('approach_pose', {})

            if not isinstance(pose, dict):
                return

            if not bool(pose.get('valid', False)):
                return

            shelf_id = self._extract_shelf_id_for_current_shelf(shelf)

            now_stamp = self.get_clock().now().nanoseconds / 1e9
            if stamp is None:
                stamp = now_stamp

            x = float(pose.get('x', 0.0))
            y = float(pose.get('y', 0.0))
            yaw = float(pose.get('yaw', 0.0))

            msg_data = {
                'shelf_id': shelf_id,
                'id': shelf_id,
                'x': x,
                'y': y,
                'yaw': yaw,
                'shelf_front_yaw': yaw,
                'stamp': float(stamp),
                'publish_stamp': float(now_stamp),
                'source': str(source),

                # Compatibility for nodes that expect nested pose format.
                'pose': {
                    'position': {
                        'x': x,
                        'y': y,
                        'z': float(pose.get('z', 0.0)),
                    },
                    'yaw': yaw,
                },

                'current_map_pose': {
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'valid': bool(pose.get('valid', True)),
                },

                'approach_pose': {
                    'x': float(approach.get('x', 0.0)) if isinstance(approach, dict) else 0.0,
                    'y': float(approach.get('y', 0.0)) if isinstance(approach, dict) else 0.0,
                    'yaw': float(approach.get('yaw', 0.0)) if isinstance(approach, dict) else 0.0,
                    'valid': bool(approach.get('valid', False)) if isinstance(approach, dict) else False,
                },

                'status': str(shelf.get('status', '')),
                'confidence': float(shelf.get('confidence', 1.0)),
                'last_seen': float(shelf.get('last_seen', stamp)),
            }

            msg = String()
            msg.data = json.dumps(msg_data, ensure_ascii=False)

            self._last_current_shelf_msg = msg
            self.current_shelf_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'publish_current_shelf failed: {e}')

    def republish_current_shelf(self):
        """
        Periodically republish the last shelf pose so late subscribers and
        ros2 topic echo --once can receive it even with VOLATILE QoS.
        """
        try:
            if self._last_current_shelf_msg is not None:
                self.current_shelf_pub.publish(self._last_current_shelf_msg)
        except Exception as e:
            self.get_logger().warn(f'republish_current_shelf failed: {e}')
'''

    # Put helper before write_shelf_pose if possible.
    idx = s.find("    def write_shelf_pose")
    if idx < 0:
        # Fallback: insert before Query helpers or before first method after __init__.
        markers = [
            "    # ------------------------------------------------------------------\n    # Query helpers",
            "    def has_front_anchor",
            "    def on_",
        ]
        for marker in markers:
            idx = s.find(marker)
            if idx >= 0:
                break

    if idx < 0:
        raise SystemExit("[ERROR] cannot find insertion point for helper methods")

    print("[OK] added publish_current_shelf() + republish_current_shelf()")
    return s[:idx] + helper + "\n" + s[idx:]


def patch_write_shelf_pose(s: str) -> str:
    if "self.publish_current_shelf(shelf, stamp" in s:
        print("[SKIP] write_shelf_pose already calls publish_current_shelf()")
        return s

    m = get_function_block(s, "write_shelf_pose")
    if not m:
        raise SystemExit("[ERROR] cannot find write_shelf_pose()")

    block = m.group(0)
    new_block = block

    call = (
        "\n"
        "        # Publish latest current shelf for BT / OSD / OAK package search.\n"
        "        self.publish_current_shelf(\n"
        "            shelf,\n"
        "            stamp,\n"
        "            locals().get('source', 'semantic_registrar'),\n"
        "        )\n"
    )

    # Best insertion point: after shelf['last_seen'] = stamp.
    mm = re.search(r"^        shelf\[['\"]last_seen['\"]\]\s*=\s*stamp\s*$", new_block, flags=re.M)
    if mm:
        pos = mm.end()
        new_block = new_block[:pos] + call + new_block[pos:]
        print("[OK] inserted publish call after shelf['last_seen'] = stamp")
        return s[:m.start()] + new_block + s[m.end():]

    # Fallback: after current_map_pose block if possible.
    mm = re.search(r"^        shelf\[['\"]current_map_pose['\"]\]\s*=\s*\{.*?\n        \}\s*$", new_block, flags=re.M | re.S)
    if mm:
        pos = mm.end()
        new_block = new_block[:pos] + call + new_block[pos:]
        print("[OK] inserted publish call after current_map_pose")
        return s[:m.start()] + new_block + s[m.end():]

    raise SystemExit("[ERROR] cannot find insertion point inside write_shelf_pose()")


def patch_bringup_yaml():
    if not BRINGUP_YAML.exists():
        print(f"[SKIP] bringup YAML not found: {BRINGUP_YAML}")
        return

    backup(BRINGUP_YAML)

    cfg = yaml.safe_load(BRINGUP_YAML.read_text())
    modules = cfg.setdefault("modules", {})

    # Your task enable list uses semantic_registrar, so prefer this key.
    candidates = ["semantic_registrar", "semantic_registrar_node"]

    mod = None
    mod_key = None
    for k in candidates:
        if isinstance(modules.get(k), dict):
            mod = modules[k]
            mod_key = k
            break

    if mod is None:
        print("[WARN] semantic_registrar module not found in bringup_modes.yaml; source defaults are enough")
        BRINGUP_YAML.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
        return

    params = mod.setdefault("parameters", [])
    param_dict = None
    for item in params:
        if isinstance(item, dict):
            param_dict = item
            break

    if param_dict is None:
        param_dict = {}
        params.append(param_dict)

    param_dict.update({
        "current_shelf_topic": "/semantic/current_shelf",
        "current_shelf_republish_period_sec": 0.5,
    })

    BRINGUP_YAML.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    print(f"[OK] patched bringup module: {mod_key}")


def main():
    print("============================================================")
    print("[PATCH] semantic_registrar publishes /semantic/current_shelf")
    print("============================================================")

    node_file = find_node_file()
    print(f"[INFO] node file: {node_file}")

    backup(node_file)
    s = node_file.read_text(errors="ignore")

    s = ensure_imports(s)
    s = patch_init(s)
    s = add_helper_methods(s)
    s = patch_write_shelf_pose(s)

    node_file.write_text(s)
    print(f"[OK] patched node file: {node_file}")

    patch_bringup_yaml()

    print("============================================================")
    print("[DONE]")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot_logistics_semantic robot2_bringup")
    print("  source install/setup.bash")
    print("============================================================")


if __name__ == "__main__":
    main()
