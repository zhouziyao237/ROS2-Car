#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

URDF = Path("/host-home/src/Diverse/robot2/urdf/robot2.urdf")
NODE = Path("/host-home/src/target_detection/oak_package_search/oak_package_search/oak_package_search_node.py")
BRINGUP_YAML = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")


def backup(p: Path):
    if p.exists():
        bak = p.with_suffix(p.suffix + f".bak_direct_oak_yaw_{STAMP}")
        shutil.copy2(p, bak)
        print(f"[BACKUP] {p} -> {bak}")


def ensure_imports(s: str) -> str:
    if "from rclpy.duration import Duration" not in s:
        s = s.replace("import rclpy\n", "import rclpy\nfrom rclpy.duration import Duration\n", 1)

    if "from typing import" in s:
        m = re.search(r"from typing import ([^\n]+)", s)
        if m:
            names = [x.strip() for x in m.group(1).split(",")]
            for name in ["Optional", "Tuple"]:
                if name not in names:
                    names.append(name)
            new_line = "from typing import " + ", ".join(names)
            s = s[:m.start()] + new_line + s[m.end():]
    else:
        s = "from typing import Optional, Tuple\n" + s

    return s


def replace_function(src: str, func_name: str, new_func: str) -> str:
    pattern = rf"\n    def {func_name}\(.*?\n(?=\n    def |\nclass |\Z)"
    m = re.search(pattern, src, flags=re.S)
    if not m:
        raise SystemExit(f"[ERROR] cannot find function: {func_name}")
    return src[:m.start()] + "\n" + new_func.rstrip() + "\n" + src[m.end():]


def patch_urdf_remove_forward_frames():
    if not URDF.exists():
        print(f"[SKIP] URDF not found: {URDF}")
        return

    s = URDF.read_text(errors="ignore")
    backup(URDF)

    # Remove our marked OAK application block.
    s2 = re.sub(
        r"\n\s*<!--\s*OAK application TF frames.*?<!--\s*/OAK application TF frames\s*-->\s*\n",
        "\n",
        s,
        flags=re.S,
    )

    # Remove unmarked oak_camera_link / oak_camera_forward_link block if it exists.
    s2 = re.sub(
        r"\n\s*<link name=\"oak_camera_link\"\s*/>\s*"
        r"<joint name=\"oak_camera_link_joint\" type=\"fixed\">.*?</joint>\s*"
        r"<link name=\"oak_camera_forward_link\"\s*/>\s*"
        r"<joint name=\"oak_camera_forward_joint\" type=\"fixed\">.*?</joint>\s*",
        "\n",
        s2,
        flags=re.S,
    )

    URDF.write_text(s2)
    print("[OK] removed oak_camera_link / oak_camera_forward_link from URDF")
    print("[OK] now oak_package_search will use oak-cam-Link directly")


def patch_node_direct_yaw():
    if not NODE.exists():
        raise SystemExit(f"[ERROR] node not found: {NODE}")

    s = NODE.read_text(errors="ignore")
    backup(NODE)
    s = ensure_imports(s)

    # Replace get_oak_yaw_from_tf so it uses only the camera frame orientation.
    new_func = r'''
    def get_oak_yaw_from_tf(self) -> Tuple[bool, Optional[float], Optional[str]]:
        fixed_frame = str(self.get_parameter('fixed_frame').value)
        oak_frame = str(self.get_parameter('oak_camera_frame').value)

        try:
            tf_cam = self.tf_buffer.lookup_transform(
                fixed_frame,
                oak_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            return False, None, f'tf error: {e}'

        q = tf_cam.transform.rotation

        # Direct-yaw mode:
        # Assumption: oak_camera_frame is a robot-style camera body frame,
        # and its +X axis is the OAK looking direction.
        # Therefore yaw(map -> oak_camera_frame) is the OAK forward yaw.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        return True, yaw, None
'''

    s = replace_function(s, "get_oak_yaw_from_tf", new_func)
    print("[OK] patched get_oak_yaw_from_tf(): direct yaw from oak_camera_frame")

    NODE.write_text(s)


def patch_yaml_direct_frame():
    if not BRINGUP_YAML.exists():
        print(f"[SKIP] bringup YAML not found: {BRINGUP_YAML}")
        return

    backup(BRINGUP_YAML)

    cfg = yaml.safe_load(BRINGUP_YAML.read_text())
    modules = cfg.setdefault("modules", {})
    mod = modules.get("oak_package_search")

    if not isinstance(mod, dict):
        raise SystemExit("[ERROR] oak_package_search module not found in bringup_modes.yaml")

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
        "fixed_frame": "map",

        # Directly use the real OAK mechanical link.
        # This assumes oak-cam-Link red X axis points to the actual camera front.
        "oak_camera_frame": "oak-cam-Link",

        # Kept only for backward compatibility. New code no longer uses it.
        "oak_forward_frame": "",

        "align_only": False,
        "yaw_control_sign": 1.0,
        "yaw_deadband_deg": 2.0,
        "yaw_settle_count_required": 3,
        "stop_publish_repeat": 5,
        "min_wz": 0.0,
        "max_wz": 0.16,
        "kp_wz": 0.6,
        "align_yaw_tolerance_deg": 5.0,
        "scan_start_yaw_tolerance_deg": 5.0,
    })

    BRINGUP_YAML.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    print("[OK] bringup_modes.yaml uses oak-cam-Link directly")


def main():
    print("============================================================")
    print("[PATCH] Simplify OAK yaw: use oak-cam-Link directly")
    print("============================================================")
    patch_urdf_remove_forward_frames()
    patch_node_direct_yaw()
    patch_yaml_direct_frame()
    print("============================================================")
    print("[DONE]")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot2 robot2_bringup oak_package_search")
    print("  source install/setup.bash")
    print("============================================================")


if __name__ == '__main__':
    main()
