from pathlib import Path
import re
import time

SEARCH_ROOTS = [
    Path("/host-home/src/Navigation"),
    Path("/host-home/src/Diverse"),
    Path("/host-home/src/target_detection"),
]

stamp = time.strftime("%Y%m%d_%H%M%S")

# D435 / RealSense 目标 frame 配置
TARGET = {
    "base_frame_id": "d435-cam-Link",
    "depth_frame_id": "camera_depth_frame",
    "color_frame_id": "camera_color_frame",
    "depth_optical_frame_id": "camera_depth_optical_frame",
    "color_optical_frame_id": "camera_color_optical_frame",
    "infra1_frame_id": "camera_infra1_frame",
    "infra1_optical_frame_id": "camera_infra1_optical_frame",
    "infra2_frame_id": "camera_infra2_frame",
    "infra2_optical_frame_id": "camera_infra2_optical_frame",
}

KEYWORDS = [
    "realsense",
    "d435",
    "rs_launch",
    "camera_link",
    "base_frame_id",
    "color_optical_frame_id",
    "depth_optical_frame_id",
]

def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_d435_frame_config_{stamp}")
    bak.write_text(path.read_text(errors="ignore"))
    print(f"[BACKUP] {path} -> {bak}")

def is_candidate(path: Path) -> bool:
    if path.suffix not in [".py", ".yaml", ".yml", ".xml"]:
        return False

    name = str(path).lower()
    if any(k in name for k in ["realsense", "d435", "camera", "sensor", "bringup"]):
        return True

    try:
        s = path.read_text(errors="ignore").lower()
    except Exception:
        return False

    return any(k.lower() in s for k in KEYWORDS)

def patch_yaml_like(s: str) -> tuple[str, bool]:
    changed = False

    # YAML style:
    # base_frame_id: camera_link
    # base_frame_id: "camera_link"
    for key, value in TARGET.items():
        pattern = rf"(^\s*{re.escape(key)}\s*:\s*)(['\"]?)([^'\"\n#]+)(['\"]?)"
        repl = rf"\1\2{value}\4"
        s2 = re.sub(pattern, repl, s, flags=re.M)
        if s2 != s:
            s = s2
            changed = True

    # ROS parameter style:
    # - base_frame_id:=camera_link
    # base_frame_id:=camera_link
    for key, value in TARGET.items():
        pattern = rf"({re.escape(key)}\s*:=\s*)([^,\s\]}}'\"#]+)"
        repl = rf"\1{value}"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

    return s, changed

def patch_python_launch(s: str) -> tuple[str, bool]:
    changed = False

    # Dict style:
    # 'base_frame_id': 'camera_link'
    # "base_frame_id": "camera_link"
    for key, value in TARGET.items():
        pattern = rf"(['\"]{re.escape(key)}['\"]\s*:\s*)(['\"])([^'\"]+)(['\"])"
        repl = rf"\1\2{value}\4"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

    # DeclareLaunchArgument style:
    # DeclareLaunchArgument('base_frame_id', default_value='camera_link')
    for key, value in TARGET.items():
        pattern = rf"(DeclareLaunchArgument\(\s*['\"]{re.escape(key)}['\"]\s*,\s*default_value\s*=\s*)(['\"])([^'\"]+)(['\"])"
        repl = rf"\1\2{value}\4"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

    # Launch argument list style:
    # 'base_frame_id:=camera_link'
    for key, value in TARGET.items():
        pattern = rf"(['\"]{re.escape(key)}:=)([^'\"]+)(['\"])"
        repl = rf"\1{value}\3"
        s2 = re.sub(pattern, repl, s)
        if s2 != s:
            s = s2
            changed = True

    return s, changed

def ensure_override_yaml():
    config_dir = Path("/host-home/src/Navigation/robot2_bringup/config")
    if not config_dir.exists():
        print(f"[WARN] config dir not found, skip override yaml: {config_dir}")
        return

    out = config_dir / "d435_frame_ids.yaml"

    content = """# D435 / RealSense frame override
# Mechanical mount link is defined in robot2.urdf:
#   base_link -> d435-cam-Link
#
# RealSense should attach its internal camera frames under d435-cam-Link.
# Do not use camera_link as a detached root frame.

realsense2_camera:
  ros__parameters:
    publish_tf: true
    tf_publish_rate: 0.0

    base_frame_id: d435-cam-Link

    depth_frame_id: camera_depth_frame
    color_frame_id: camera_color_frame
    depth_optical_frame_id: camera_depth_optical_frame
    color_optical_frame_id: camera_color_optical_frame

    infra1_frame_id: camera_infra1_frame
    infra1_optical_frame_id: camera_infra1_optical_frame
    infra2_frame_id: camera_infra2_frame
    infra2_optical_frame_id: camera_infra2_optical_frame
"""

    if out.exists():
        backup(out)

    out.write_text(content)
    print(f"[WRITE] {out}")

def main():
    print("============================================================")
    print("[STEP 1] Create D435 frame override yaml")
    print("============================================================")
    ensure_override_yaml()

    print("")
    print("============================================================")
    print("[STEP 2] Patch existing D435 / RealSense config and launch files")
    print("============================================================")

    candidates = []
    for root in SEARCH_ROOTS:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and is_candidate(path):
                candidates.append(path)

    print(f"[INFO] candidate files: {len(candidates)}")

    changed_files = []

    for path in candidates:
        try:
            s = path.read_text(errors="ignore")
        except Exception:
            continue

        old = s

        s, c1 = patch_yaml_like(s)
        s, c2 = patch_python_launch(s)

        if s != old:
            backup(path)
            path.write_text(s)
            changed_files.append(path)
            print(f"[OK] patched {path}")

    print("")
    print("============================================================")
    print("[STEP 3] Summary")
    print("============================================================")

    if changed_files:
        print("[OK] patched files:")
        for p in changed_files:
            print(f"  - {p}")
    else:
        print("[WARN] No existing launch/config file was patched.")
        print("[WARN] You may need to include this yaml manually:")
        print("       /host-home/src/Navigation/robot2_bringup/config/d435_frame_ids.yaml")

    print("")
    print("[DONE]")
    print("Target D435 frame config:")
    for k, v in TARGET.items():
        print(f"  {k}: {v}")

    print("")
    print("Next commands:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot2_bringup robot2")
    print("  source install/setup.bash")

if __name__ == "__main__":
    main()
