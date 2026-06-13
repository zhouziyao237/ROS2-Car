#!/usr/bin/env python3
from pathlib import Path
import time
import re
import shutil
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

URDF = Path("/host-home/src/Diverse/robot2/urdf/robot2.urdf")
BRINGUP_YAML = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")
STACK_LAUNCH = Path("/host-home/src/Navigation/robot2_bringup/launch/oak_package_search_stack.launch.py")
INSTALL_STACK_LAUNCH = Path("/host-home/install/robot2_bringup/share/robot2_bringup/launch/oak_package_search_stack.launch.py")

def backup(p: Path):
    if p.exists():
        bak = p.with_suffix(p.suffix + f".bak_oak_inline_{STAMP}")
        shutil.copy2(p, bak)
        print(f"[BACKUP] {p} -> {bak}")
        return bak
    return None

def patch_urdf():
    if not URDF.exists():
        raise SystemExit(f"[ERROR] URDF not found: {URDF}")

    s = URDF.read_text(errors="ignore")
    backup(URDF)

    if "oak-cam-Link" not in s:
        raise SystemExit("[ERROR] oak-cam-Link not found in robot2.urdf. Cannot attach OAK application frames.")

    # Remove previous marked OAK application frame block if this script was run before.
    s2 = re.sub(
        r"\n\s*<!--\s*OAK application TF frames.*?<!--\s*/OAK application TF frames\s*-->\s*\n",
        "\n",
        s,
        flags=re.S,
    )
    if s2 != s:
        s = s2
        print("[OK] removed previous marked OAK application TF block")

    if "oak_camera_link" in s or "oak_camera_forward_link" in s:
        print("[WARN] oak_camera_link or oak_camera_forward_link already exists in URDF.")
        print("[WARN] To avoid corrupting URDF, script will not add duplicate links.")
        print("[WARN] Please inspect with:")
        print(f"       grep -n \"oak_camera_link\\|oak_camera_forward_link\" {URDF}")
        URDF.write_text(s)
        return

    block = r'''
  <!-- OAK application TF frames.
       oak-cam-Link is the mechanical exported link.
       oak_camera_link is the ROS application frame used by oak_parcel_pose / oak_package_search.
       oak_camera_forward_link is a virtual point 1m in front of OAK for yaw calculation.

       Current setting:
       oak-cam-Joint has yaw -90deg in the exported URDF.
       This joint uses +90deg so oak_camera_link +X becomes the effective OAK forward direction.
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

    # Insert after oak-cam-Joint block.
    pattern = r"(<joint\s+name=\"oak-cam-Joint\"\s+type=\"fixed\">.*?</joint>)"
    m = re.search(pattern, s, flags=re.S)
    if not m:
        raise SystemExit("[ERROR] oak-cam-Joint block not found. Cannot insert OAK frames safely.")

    insert_at = m.end()
    s = s[:insert_at] + block + s[insert_at:]
    URDF.write_text(s)
    print(f"[OK] added oak_camera_link and oak_camera_forward_link into {URDF}")

def patch_bringup_yaml():
    if not BRINGUP_YAML.exists():
        raise SystemExit(f"[ERROR] bringup_modes.yaml not found: {BRINGUP_YAML}")

    backup(BRINGUP_YAML)

    with open(BRINGUP_YAML, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if not isinstance(cfg, dict):
        raise SystemExit("[ERROR] bringup_modes.yaml is not a YAML dictionary")

    modules = cfg.setdefault("modules", {})
    modes = cfg.setdefault("modes", {})

    # OAK camera driver.
    modules["oak_vision_driver"] = {
        "type": "include",
        "package": "oak_vision_driver",
        "launch": "oak_vision_driver.launch.py",
        "delay": 13.0,
        "arguments": {
            "enable_rgb": "true",
            "enable_depth": "true",
            "enable_yolo": "false",
            "enable_stereo_fusion": "false",
            "sensor_resolution": "1080p",
            "rgb_output": "video",
            "video_width": "1920",
            "video_height": "1080",
            "fps": "10.0",
            "force_usb2": "true",
        },
    }

    # DataMatrix + depth position detector.
    modules["oak_parcel_pose"] = {
        "type": "include",
        "package": "oak_parcel_pose",
        "launch": "oak_parcel_pose.launch.py",
        "delay": 16.0,
    }

    # Search behavior / rotation / sweep / code matching.
    modules["oak_package_search"] = {
        "type": "node",
        "package": "oak_package_search",
        "executable": "oak_package_search_node",
        "name": "oak_package_search_node",
        "output": "screen",
        "delay": 17.0,
        "respawn": True,
        "respawn_delay": 2.0,
        "parameters": [
            "/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml",
            {
                "request_topic": "/target_detection/package_search/request",
                "status_topic": "/target_detection/package_search/status",
                "datamatrix_topic": "/oak/datamatrix_3d_result_json",
                "cmd_vel_topic": "/cmd_vel",
                "fixed_frame": "map",
                "oak_camera_frame": "oak_camera_link",
                "oak_forward_frame": "oak_camera_forward_link",

                # Real behavior. It will not move until it receives a package_search request.
                "enable_motion": True,

                "match_only_in_scan": True,
                "ignore_detection_before_request": True,
                "min_confirm_matches": 2,
                "min_search_time_before_match_sec": 0.5,
                "datamatrix_fresh_timeout_sec": 1.2,

                "scan_range_deg": 25.0,
                "scan_speed_wz": 0.10,
                "scan_timeout_sec": 30.0,
                "max_scan_passes": 2,

                "align_yaw_tolerance_deg": 3.0,
                "scan_start_yaw_tolerance_deg": 3.0,
                "kp_wz": 1.0,
                "min_wz": 0.06,
                "max_wz": 0.25,

                "require_tf_for_alignment": True,
                "allow_no_tf_in_dry_run": False,
                "tf_failure_timeout_sec": 8.0,
            },
        ],
    }

    # Enable OAK modules in task mode.
    task = modes.setdefault("task", {})
    enabled = task.setdefault("enable", [])

    for name in ["oak_vision_driver", "oak_parcel_pose", "oak_package_search"]:
        if name not in enabled:
            enabled.append(name)
            print(f"[OK] added {name} to modes.task.enable")

    with open(BRINGUP_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)

    print(f"[OK] patched {BRINGUP_YAML}")

def disable_stack_launch():
    if STACK_LAUNCH.exists():
        backup(STACK_LAUNCH)
        disabled = STACK_LAUNCH.with_name("oak_package_search_stack.launch.py.disabled")
        if disabled.exists():
            disabled.unlink()
        STACK_LAUNCH.rename(disabled)
        print(f"[OK] disabled {STACK_LAUNCH} -> {disabled}")
    else:
        print("[SKIP] source oak_package_search_stack.launch.py not found")

    if INSTALL_STACK_LAUNCH.exists():
        bak = INSTALL_STACK_LAUNCH.with_suffix(INSTALL_STACK_LAUNCH.suffix + f".bak_oak_inline_{STAMP}")
        shutil.copy2(INSTALL_STACK_LAUNCH, bak)
        INSTALL_STACK_LAUNCH.unlink()
        print(f"[OK] removed installed stack launch: {INSTALL_STACK_LAUNCH}")
        print(f"[BACKUP] installed stack launch -> {bak}")
    else:
        print("[SKIP] installed oak_package_search_stack.launch.py not found")

def main():
    print("============================================================")
    print("[PATCH] YAML-driven OAK unified bringup")
    print("============================================================")
    patch_urdf()
    patch_bringup_yaml()
    disable_stack_launch()
    print("============================================================")
    print("[DONE] Patch finished.")
    print("Next:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot2 robot2_bringup oak_vision_driver oak_parcel_pose oak_package_search")
    print("  source install/setup.bash")
    print("============================================================")

if __name__ == "__main__":
    main()
