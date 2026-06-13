#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

NODE = Path("/host-home/src/target_detection/oak_package_search/oak_package_search/oak_package_search_node.py")
URDF = Path("/host-home/src/Diverse/robot2/urdf/robot2.urdf")
BRINGUP_YAML = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")


def backup(p: Path):
    if p.exists():
        bak = p.with_suffix(p.suffix + f".bak_align_only_v2_{STAMP}")
        shutil.copy2(p, bak)
        print(f"[BACKUP] {p} -> {bak}")


def replace_function(src: str, func_name: str, new_func: str) -> str:
    pattern = rf"\n    def {func_name}\(.*?\n(?=\n    def |\nclass |\Z)"
    m = re.search(pattern, src, flags=re.S)
    if not m:
        raise SystemExit(f"[ERROR] cannot find function: {func_name}")
    return src[:m.start()] + "\n" + new_func.rstrip() + "\n" + src[m.end():]


def ensure_typing_import(src: str) -> str:
    # The replacement function uses Tuple and Optional.
    if "from typing import" not in src:
        return "from typing import Any, Dict, List, Optional, Tuple\n" + src

    line_match = re.search(r"from typing import ([^\n]+)", src)
    if not line_match:
        return src

    names = [x.strip() for x in line_match.group(1).split(",")]
    for n in ["Optional", "Tuple"]:
        if n not in names:
            names.append(n)

    new_line = "from typing import " + ", ".join(names)
    return src[:line_match.start()] + new_line + src[line_match.end():]


def patch_urdf_back_to_working_link():
    if not URDF.exists():
        print(f"[SKIP] URDF not found: {URDF}")
        return

    s = URDF.read_text(errors="ignore")
    backup(URDF)

    if "oak-cam-Link" not in s:
        raise SystemExit("[ERROR] oak-cam-Link not found in URDF")

    # Remove previous marked OAK application block.
    s = re.sub(
        r"\n\s*<!--\s*OAK application TF frames.*?<!--\s*/OAK application TF frames\s*-->\s*\n",
        "\n",
        s,
        flags=re.S,
    )

    # Remove possible unmarked duplicate block.
    s = re.sub(
        r"\n\s*<link name=\"oak_camera_link\"\s*/>\s*"
        r"<joint name=\"oak_camera_link_joint\" type=\"fixed\">.*?</joint>\s*"
        r"<link name=\"oak_camera_forward_link\"\s*/>\s*"
        r"<joint name=\"oak_camera_forward_joint\" type=\"fixed\">.*?</joint>\s*",
        "\n",
        s,
        flags=re.S,
    )

    block = r'''
  <!-- OAK application TF frames.
       Working convention:
       oak_camera_link is attached to oak-cam-Link.
       oak_camera_forward_link is 1m in front of oak_camera_link along +X.
       Do not use temporary static_transform_publisher for these frames after this.
  -->
  <link name="oak_camera_link" />
  <joint name="oak_camera_link_joint" type="fixed">
    <parent link="oak-cam-Link" />
    <child link="oak_camera_link" />
    <origin xyz="0 0 0" rpy="0 0 1.57079632679489" />
  </joint>

  <link name="oak_camera_forward_link" />
  <joint name="oak_camera_forward_joint" type="fixed">
    <parent link="oak_camera_link" />
    <child link="oak_camera_forward_link" />
    <origin xyz="1.0 0.0 0.0" rpy="0 0 0" />
  </joint>
  <!-- /OAK application TF frames -->
'''

    m = re.search(r"(<joint\s+name=\"oak-cam-Joint\"\s+type=\"fixed\">.*?</joint>)", s, flags=re.S)
    if not m:
        raise SystemExit("[ERROR] oak-cam-Joint block not found in URDF")

    s = s[:m.end()] + block + s[m.end():]
    URDF.write_text(s)
    print("[OK] restored OAK URDF TF to working convention")


def insert_declare_params(src: str) -> str:
    if "self.declare_parameter('align_only'" in src or 'self.declare_parameter("align_only"' in src:
        print("[SKIP] align_only parameters already exist")
        return src

    insert = """
        # Alignment-only test mode.
        # true: stop after OAK faces the shelf; do not enter scan.
        # false: normal package search, align -> scan start -> sweep.
        self.declare_parameter('align_only', False)

        # Yaw control tuning.
        # If chassis rotates opposite to the reducing-error direction, set yaw_control_sign to -1.0.
        self.declare_parameter('yaw_control_sign', 1.0)

        # Dead zone / stop latch.
        # Inside max(align_yaw_tolerance_deg, yaw_deadband_deg), command must be zero.
        self.declare_parameter('yaw_deadband_deg', 2.0)
        self.declare_parameter('yaw_settle_count_required', 3)
        self.declare_parameter('stop_publish_repeat', 5)
"""

    # Insert after the last declare_parameter line before "tick_hz =" if possible.
    tick = src.find("tick_hz =")
    if tick == -1:
        tick = src.find("self.create_timer")
    if tick == -1:
        raise SystemExit("[ERROR] cannot find insertion point near tick_hz/create_timer")

    before = src[:tick]
    after = src[tick:]

    matches = list(re.finditer(r"^        self\.declare_parameter\(.*\)\s*$", before, flags=re.M))
    if not matches:
        raise SystemExit("[ERROR] cannot find any self.declare_parameter line")

    pos = matches[-1].end()
    src = before[:pos] + insert + before[pos:] + after
    print("[OK] inserted align_only / deadband declarations")
    return src


def insert_settle_counter(src: str) -> str:
    if "self._yaw_settle_count" in src:
        print("[SKIP] yaw settle counter already exists")
        return src

    # Prefer inserting after found_result.
    m = re.search(r"^        self\.found_result.*$", src, flags=re.M)
    if m:
        pos = m.end()
        return src[:pos] + "\n\n        # Alignment settle counter.\n        self._yaw_settle_count = 0" + src[pos:]

    # Fallback: after state_start.
    m = re.search(r"^        self\.state_start.*$", src, flags=re.M)
    if m:
        pos = m.end()
        return src[:pos] + "\n\n        # Alignment settle counter.\n        self._yaw_settle_count = 0" + src[pos:]

    raise SystemExit("[ERROR] cannot find place to insert _yaw_settle_count")


def patch_node():
    if not NODE.exists():
        raise SystemExit(f"[ERROR] node file not found: {NODE}")

    s = NODE.read_text(errors="ignore")
    backup(NODE)

    s = ensure_typing_import(s)
    s = insert_declare_params(s)
    s = insert_settle_counter(s)

    # Replace publish_stop.
    publish_stop_func = r'''
    def publish_stop(self) -> None:
        if not bool(self.get_parameter('enable_motion').value):
            return
        repeat = int(self.get_parameter('stop_publish_repeat').value) if self.has_parameter('stop_publish_repeat') else 5
        repeat = max(1, repeat)
        for _ in range(repeat):
            self.cmd_pub.publish(Twist())
'''
    s = replace_function(s, "publish_stop", publish_stop_func)
    print("[OK] replaced publish_stop()")

    # Patch set_state if possible.
    set_state_func = r'''
    def set_state(self, new_state: str, reason: str) -> None:
        old = self.state
        self.state = new_state
        self.state_start = self.now_sec()
        self._yaw_settle_count = 0
        self.publish_stop()
        self.get_logger().info(f'state: {old} -> {new_state}, {reason}')
        self.publish_status(new_state.lower(), reason)
'''
    s = replace_function(s, "set_state", set_state_func)
    print("[OK] replaced set_state()")

    # Replace yaw_control_to_target with deadband version.
    yaw_control_func = r'''
    def yaw_control_to_target(self, target_yaw: float, tolerance_deg: float) -> Tuple[bool, Optional[str], Optional[float], Optional[float]]:
        ok, current_yaw, err_msg = self.get_oak_yaw_from_tf()
        if not ok:
            self.publish_stop()
            return False, err_msg, None, None

        yaw_error = normalize_angle(target_yaw - current_yaw)
        tol = math.radians(float(tolerance_deg))
        deadband = math.radians(float(self.get_parameter('yaw_deadband_deg').value))
        stop_zone = max(tol, deadband)

        # Stop zone must be checked before min_wz.
        # Otherwise min_wz can force a non-zero angular velocity near the target.
        if abs(yaw_error) <= stop_zone:
            self._yaw_settle_count += 1
            self.publish_stop()

            required = int(self.get_parameter('yaw_settle_count_required').value)
            required = max(1, required)

            if self._yaw_settle_count >= required:
                return True, None, current_yaw, yaw_error

            return False, None, current_yaw, yaw_error

        self._yaw_settle_count = 0

        kp = float(self.get_parameter('kp_wz').value)
        min_wz = abs(float(self.get_parameter('min_wz').value))
        max_wz = abs(float(self.get_parameter('max_wz').value))
        control_sign = float(self.get_parameter('yaw_control_sign').value)

        wz = control_sign * kp * yaw_error
        wz = clamp(wz, -max_wz, max_wz)

        # Apply min_wz only outside the dead zone.
        if min_wz > 0.0 and abs(wz) < min_wz:
            sign_source = wz if abs(wz) > 1e-6 else control_sign * yaw_error
            wz = math.copysign(min_wz, sign_source)

        self.publish_wz(wz)
        return False, None, current_yaw, yaw_error
'''
    s = replace_function(s, "yaw_control_to_target", yaw_control_func)
    print("[OK] replaced yaw_control_to_target()")

    # Patch ALIGN transition.
    if "align_only=true, scan disabled" not in s:
        patterns = [
            r"""if aligned:\s*\n\s*self\.prepare_scan\(\)\s*\n\s*self\.set_state\(self\.MOVE_TO_SCAN_START,\s*['"]OAK aligned to shelf front['"]\)\s*\n\s*return""",
            r"""if aligned:\s*\n\s*self\.set_state\(self\.MOVE_TO_SCAN_START,\s*['"]OAK aligned to shelf front['"]\)\s*\n\s*return""",
        ]

        replacement = """if aligned:
                if bool(self.get_parameter('align_only').value):
                    self.publish_stop()
                    self.publish_status(
                        'aligned',
                        'OAK aligned to shelf front; align_only=true, scan disabled',
                        {'current_oak_yaw': current_yaw, 'yaw_error': yaw_error},
                    )
                    self.get_logger().info('OAK aligned; align_only=true, stop before scan')
                    self.state = self.IDLE
                    self.state_start = self.now_sec()
                    return

                self.prepare_scan()
                self.set_state(self.MOVE_TO_SCAN_START, 'OAK aligned to shelf front')
            return"""

        patched = False
        for pat in patterns:
            s2, n = re.subn(pat, replacement, s, count=1, flags=re.S)
            if n:
                s = s2
                patched = True
                print("[OK] patched ALIGN transition with align_only")
                break

        if not patched:
            print("[WARN] Could not auto-patch ALIGN transition.")
            print("[WARN] Search manually:")
            print("       grep -n \"if aligned\\|prepare_scan\\|MOVE_TO_SCAN_START\" " + str(NODE))
    else:
        print("[SKIP] ALIGN transition already has align_only")

    NODE.write_text(s)
    print(f"[DONE] patched node: {NODE}")


def patch_bringup_yaml():
    if not BRINGUP_YAML.exists():
        print(f"[SKIP] bringup yaml not found: {BRINGUP_YAML}")
        return

    backup(BRINGUP_YAML)

    cfg = yaml.safe_load(BRINGUP_YAML.read_text())
    modules = cfg.setdefault("modules", {})
    mod = modules.get("oak_package_search")

    if not isinstance(mod, dict):
        print("[SKIP] oak_package_search module not found in bringup_modes.yaml")
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
        # Default formal mode: full search. For testing, set true at runtime.
        "align_only": False,

        "yaw_control_sign": 1.0,
        "yaw_deadband_deg": 2.0,
        "yaw_settle_count_required": 3,
        "stop_publish_repeat": 5,

        "align_yaw_tolerance_deg": 5.0,
        "scan_start_yaw_tolerance_deg": 5.0,
        "min_wz": 0.0,
        "max_wz": 0.16,
        "kp_wz": 0.6,

        "request_topic": "/target_detection/package_search/request",
        "status_topic": "/target_detection/package_search/status",
        "datamatrix_topic": "/oak/datamatrix_3d_result_json",
        "cmd_vel_topic": "/cmd_vel",
        "fixed_frame": "map",
        "oak_camera_frame": "oak_camera_link",
        "oak_forward_frame": "oak_camera_forward_link",
    })

    BRINGUP_YAML.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    print("[OK] patched bringup_modes.yaml defaults")


def main():
    print("============================================================")
    print("[PATCH V2] OAK align-only test mode + deadband + URDF TF restore")
    print("============================================================")
    patch_urdf_back_to_working_link()
    patch_node()
    patch_bringup_yaml()
    print("============================================================")
    print("[DONE]")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot2 robot2_bringup oak_package_search")
    print("  source install/setup.bash")
    print("============================================================")


if __name__ == "__main__":
    main()
