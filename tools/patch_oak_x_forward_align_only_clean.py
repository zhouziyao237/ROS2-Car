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
        bak = p.with_suffix(p.suffix + f".bak_x_forward_align_only_{STAMP}")
        shutil.copy2(p, bak)
        print(f"[BACKUP] {p} -> {bak}")


def replace_function(src: str, func_name: str, new_func: str) -> str:
    pattern = rf"\n    def {func_name}\(.*?\n(?=\n    def |\nclass |\Z)"
    m = re.search(pattern, src, flags=re.S)
    if not m:
        raise SystemExit(f"[ERROR] cannot find function: {func_name}")
    return src[:m.start()] + "\n" + new_func.rstrip() + "\n" + src[m.end():]


def ensure_typing_import(src: str) -> str:
    if "from typing import" not in src:
        return "from typing import Any, Dict, List, Optional, Tuple\n" + src

    m = re.search(r"from typing import ([^\n]+)", src)
    if not m:
        return src

    names = [x.strip() for x in m.group(1).split(",")]
    for name in ["Optional", "Tuple"]:
        if name not in names:
            names.append(name)

    new_line = "from typing import " + ", ".join(names)
    return src[:m.start()] + new_line + src[m.end():]


def patch_urdf_x_forward():
    if not URDF.exists():
        raise SystemExit(f"[ERROR] URDF not found: {URDF}")

    s = URDF.read_text(errors="ignore")
    backup(URDF)

    if "oak-cam-Link" not in s:
        raise SystemExit("[ERROR] oak-cam-Link not found in robot2.urdf")

    # Remove previous marked OAK application block.
    s = re.sub(
        r"\n\s*<!--\s*OAK application TF frames.*?<!--\s*/OAK application TF frames\s*-->\s*\n",
        "\n",
        s,
        flags=re.S,
    )

    # Remove possible unmarked old oak_camera_link block.
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
       User convention:
       oak_camera_link uses robot-style camera body frame:
       +X = OAK looking direction, +Y/+Z correspond to image-plane axes.
       oak_camera_forward_link is only a virtual point for yaw calculation.
  -->
  <link name="oak_camera_link" />
  <joint name="oak_camera_link_joint" type="fixed">
    <parent link="oak-cam-Link" />
    <child link="oak_camera_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <link name="oak_camera_forward_link" />
  <joint name="oak_camera_forward_joint" type="fixed">
    <parent link="oak_camera_link" />
    <child link="oak_camera_forward_link" />
    <origin xyz="0.5 0.0 0.0" rpy="0 0 0" />
  </joint>
  <!-- /OAK application TF frames -->
'''

    m = re.search(r"(<joint\s+name=\"oak-cam-Joint\"\s+type=\"fixed\">.*?</joint>)", s, flags=re.S)
    if not m:
        raise SystemExit("[ERROR] oak-cam-Joint block not found")

    s = s[:m.end()] + block + s[m.end():]
    URDF.write_text(s)

    print("[OK] URDF restored to user convention: oak_camera_link +X is forward")
    print("[OK] oak_camera_forward_link = +X 0.5m virtual direction point")


def patch_node_clean_align_only():
    if not NODE.exists():
        raise SystemExit(f"[ERROR] node not found: {NODE}")

    s = NODE.read_text(errors="ignore")
    backup(NODE)

    s = ensure_typing_import(s)

    # Remove all previously inserted duplicate declarations for these params.
    managed_params = [
        "align_only",
        "yaw_control_sign",
        "yaw_deadband_deg",
        "yaw_settle_count_required",
        "stop_publish_repeat",
        "ki_wz",
        "kd_wz",
        "pid_integral_limit",
    ]

    lines = s.splitlines()
    out = []
    removed = []

    for line in lines:
        m = re.search(r"self\.declare_parameter\(['\"]([^'\"]+)['\"]", line)
        if m and m.group(1) in managed_params:
            removed.append(line.strip())
            continue
        out.append(line)

    s = "\n".join(out) + "\n"

    if removed:
        print("[OK] removed old/duplicate parameter declarations:")
        for line in removed:
            print("  " + line)

    # Insert one clean canonical declaration block.
    declare_block = """
        # Alignment-only test mode.
        # true: only align OAK to shelf front and stop; no scan.
        # false: normal package search: align -> scan start -> sweep.
        self.declare_parameter('align_only', False)

        # Keep geometry in URDF; only use this to flip chassis angular.z if needed.
        self.declare_parameter('yaw_control_sign', 1.0)

        # Stop zone / stop latch.
        self.declare_parameter('yaw_deadband_deg', 2.0)
        self.declare_parameter('yaw_settle_count_required', 3)
        self.declare_parameter('stop_publish_repeat', 5)
"""

    m = re.search(r"^        self\.declare_parameter\(['\"]max_wz['\"].*$", s, flags=re.M)
    if m:
        pos = m.end()
        s = s[:pos] + declare_block + s[pos:]
        print("[OK] inserted clean align_only declarations after max_wz")
    else:
        idx = s.find("tick_hz =")
        if idx < 0:
            idx = s.find("self.create_timer")
        if idx < 0:
            raise SystemExit("[ERROR] cannot find insertion point for declarations")
        s = s[:idx] + declare_block + s[idx:]
        print("[OK] inserted clean align_only declarations before timer")

    # Remove duplicate yaw settle counter assignments and ensure one exists.
    lines = s.splitlines()
    out = []
    settle_seen = False
    for line in lines:
        if line.strip().startswith("self._yaw_settle_count") and "=" in line:
            if settle_seen:
                continue
            settle_seen = True
        out.append(line)

    s = "\n".join(out) + "\n"

    if not settle_seen:
        m = re.search(r"^        self\.found_result.*$", s, flags=re.M)
        if m:
            pos = m.end()
            s = s[:pos] + "\n\n        # Alignment settle counter.\n        self._yaw_settle_count = 0" + s[pos:]
            print("[OK] inserted _yaw_settle_count after found_result")
        else:
            m = re.search(r"^        self\.state_start.*$", s, flags=re.M)
            if not m:
                raise SystemExit("[ERROR] cannot find place to insert _yaw_settle_count")
            pos = m.end()
            s = s[:pos] + "\n\n        # Alignment settle counter.\n        self._yaw_settle_count = 0" + s[pos:]
            print("[OK] inserted _yaw_settle_count after state_start")

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

        # Dead zone first. This must be before min_wz.
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

    # Add align_only stop before prepare_scan.
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
            print("[WARN] Run this to inspect manually:")
            print(f"       grep -n \"if aligned\\|prepare_scan\\|MOVE_TO_SCAN_START\" {NODE}")
    else:
        print("[OK] ALIGN transition already has align_only")

    NODE.write_text(s)
    print(f"[DONE] patched node: {NODE}")


def patch_bringup_yaml_defaults():
    if not BRINGUP_YAML.exists():
        print(f"[SKIP] bringup YAML not found: {BRINGUP_YAML}")
        return

    backup(BRINGUP_YAML)

    cfg = yaml.safe_load(BRINGUP_YAML.read_text())
    modules = cfg.setdefault("modules", {})
    mod = modules.get("oak_package_search")

    if not isinstance(mod, dict):
        print("[SKIP] oak_package_search module not found")
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

    # Remove old PID keys if they exist.
    for key in ["ki_wz", "kd_wz", "pid_integral_limit"]:
        param_dict.pop(key, None)

    param_dict.update({
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
    print("[PATCH] OAK X-forward TF + clean align_only mode")
    print("============================================================")
    patch_urdf_x_forward()
    patch_node_clean_align_only()
    patch_bringup_yaml_defaults()
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
