#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

SRC_ROOT = Path("/host-home/src")
INSTALL_ROOT = Path("/host-home/install")

def backup(p: Path) -> Path:
    b = p.with_suffix(p.suffix + f".bak_reentry_guard_{STAMP}")
    shutil.copy2(p, b)
    return b

def patch_file_text(p: Path, new_text: str, tag: str):
    old = p.read_text(errors="ignore")
    if old == new_text:
        print(f"[NO CHANGE] {tag}: {p}")
        return False
    b = backup(p)
    p.write_text(new_text)
    print(f"[OK] {tag}: {p}")
    print(f"     backup: {b}")
    return True

# ------------------------------------------------------------
# 1) Patch OAK package search node:
#    - small yaw error: do not rotate
#    - same shelf/layer recently aligned: do not rotate again
#    - search stage only reports 'aligned', then returns IDLE
# ------------------------------------------------------------

def patch_oak_package_search_node(p: Path):
    s = p.read_text(errors="ignore")
    old = s

    if "maybe_skip_search_align" in s and "publish_aligned_and_idle" in s:
        print(f"[SKIP] OAK guard already exists: {p}")
        return

    # Safer search-stage defaults.
    s = s.replace(
        "self.declare_parameter('align_yaw_tolerance_deg', 3.0)",
        "self.declare_parameter('align_yaw_tolerance_deg', 5.0)"
    )
    s = s.replace(
        "self.declare_parameter('kp_wz', 1.0)",
        "self.declare_parameter('kp_wz', 0.6)"
    )
    s = s.replace(
        "self.declare_parameter('min_wz', 0.06)",
        "self.declare_parameter('min_wz', 0.04)"
    )
    s = s.replace(
        "self.declare_parameter('max_wz', 0.30)",
        "self.declare_parameter('max_wz', 0.25)"
    )

    marker = "self.declare_parameter('max_wz', 0.25)"
    if marker in s and "search_already_ok_deg" not in s:
        s = s.replace(
            marker,
            marker + """
        # Search-stage guard.
        # This alignment is only for coarse OAK-shelf relation before scanning.
        # If OAK is already roughly parallel/facing the shelf, do not rotate again.
        self.declare_parameter('search_already_ok_deg', 8.0)
        self.declare_parameter('search_start_align_deg', 12.0)
        self.declare_parameter('same_shelf_skip_sec', 12.0)
        self.declare_parameter('search_align_only', True)"""
        )

    marker = "self.approach_yaw: Optional[float] = None"
    if marker in s and "self.last_aligned_key" not in s:
        s = s.replace(
            marker,
            marker + """

        # Re-entry guard: avoid repeatedly aligning the same shelf/layer.
        self.last_aligned_key = ''
        self.last_aligned_time = 0.0"""
        )

    methods_marker = "    def cancel(self, reason: str) -> None:\n"
    methods = '''    def current_request_key(self) -> str:
        return f'{self.shelf_id}:{self.layer_id}' if self.shelf_id or self.layer_id else self.pickup_code

    def publish_aligned_and_idle(self, reason: str, extra: Optional[Dict[str, Any]] = None) -> None:
        """Report alignment success without further rotation.

        Published status is intentionally 'aligned', so mission_btcpp_node can
        continue as if alignment completed normally.
        """
        self.publish_stop()
        key = self.current_request_key()
        if key:
            self.last_aligned_key = key
            self.last_aligned_time = self.now_sec()

        merged: Dict[str, Any] = {
            'skip_align': True,
            'reason': reason,
        }
        if extra:
            merged.update(extra)

        self.publish_status('aligned', reason, merged)
        self.state = self.IDLE
        self.state_start = self.now_sec()
        self.get_logger().info(f'align skipped/completed: {reason}, key={key}')

    def maybe_skip_search_align(self) -> Tuple[bool, str, Dict[str, Any]]:
        """Return True when search-stage alignment should be skipped.

        Search alignment only needs OAK to be roughly parallel/facing the shelf.
        Repeated small corrections can turn a usable pose into a worse pose.
        """
        now = self.now_sec()
        key = self.current_request_key()

        cooldown = float(self.get_parameter('same_shelf_skip_sec').value)
        if key and key == self.last_aligned_key and 0.0 <= now - self.last_aligned_time <= cooldown:
            return True, 'same shelf/layer aligned recently; skip repeated search align', {
                'last_aligned_key': self.last_aligned_key,
                'age_sec': now - self.last_aligned_time,
                'same_shelf_skip_sec': cooldown,
            }

        if self.target_oak_yaw is None:
            return False, '', {}

        ok, current_yaw, err_msg = self.get_oak_yaw_from_tf()
        if not ok:
            return False, '', {'tf_error': err_msg}

        yaw_error = normalize_angle(self.target_oak_yaw - current_yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))

        already_ok = float(self.get_parameter('search_already_ok_deg').value)
        start_align = float(self.get_parameter('search_start_align_deg').value)

        if yaw_error_deg <= already_ok:
            return True, 'OAK already roughly parallel/facing shelf; skip search align', {
                'current_oak_yaw': current_yaw,
                'yaw_error': yaw_error,
                'yaw_error_deg': yaw_error_deg,
                'search_already_ok_deg': already_ok,
            }

        # Deadband: between already_ok and start_align, still do not rotate.
        # This avoids boundary jitter and unnecessary small corrections.
        if yaw_error_deg < start_align:
            return True, 'OAK yaw error is within search deadband; skip small correction', {
                'current_oak_yaw': current_yaw,
                'yaw_error': yaw_error,
                'yaw_error_deg': yaw_error_deg,
                'search_start_align_deg': start_align,
            }

        return False, '', {
            'current_oak_yaw': current_yaw,
            'yaw_error': yaw_error,
            'yaw_error_deg': yaw_error_deg,
            'search_start_align_deg': start_align,
        }

'''
    if methods_marker in s and "def maybe_skip_search_align" not in s:
        s = s.replace(methods_marker, methods + methods_marker)

    req_marker = """        self.publish_status('accepted', 'request accepted')
        self.set_state(self.ALIGN, 'new package search request')"""
    req_repl = """        skip_align, skip_reason, skip_extra = self.maybe_skip_search_align()
        if skip_align:
            self.publish_aligned_and_idle(skip_reason, skip_extra)
            return

        self.publish_status('accepted', 'request accepted')
        self.set_state(self.ALIGN, 'new package search request')"""

    if req_marker in s and "skip_align, skip_reason, skip_extra = self.maybe_skip_search_align()" not in s:
        s = s.replace(req_marker, req_repl)

    align_marker = """            if aligned:
                self.prepare_scan()
                self.set_state(self.MOVE_TO_SCAN_START, 'OAK aligned to shelf front')
            return"""
    align_repl = """            if aligned:
                if bool(self.get_parameter('search_align_only').value):
                    self.publish_aligned_and_idle('OAK search alignment completed')
                    return
                self.prepare_scan()
                self.set_state(self.MOVE_TO_SCAN_START, 'OAK aligned to shelf front')
            return"""

    if align_marker in s and "search_align_only').value" not in s:
        s = s.replace(align_marker, align_repl)

    if s == old:
        print(f"[WARN] OAK node pattern not fully matched, no code change: {p}")
        print("       Send me this file if this warning appears.")
        return

    patch_file_text(p, s, "patched OAK search re-entry guard")

# ------------------------------------------------------------
# 2) Patch OAK yaml:
#    - coarse search alignment
#    - slower max wz
# ------------------------------------------------------------

def patch_oak_yaml(p: Path):
    s = p.read_text(errors="ignore")
    old = s

    replacements = {
        r"align_yaw_tolerance_deg:\s*[-0-9.]+": "align_yaw_tolerance_deg: 5.0",
        r"kp_wz:\s*[-0-9.]+": "kp_wz: 0.6",
        r"min_wz:\s*[-0-9.]+": "min_wz: 0.04",
        r"max_wz:\s*[-0-9.]+": "max_wz: 0.25",
    }

    for pat, repl in replacements.items():
        s = re.sub(pat, repl, s)

    if "search_already_ok_deg:" not in s:
        s += """

    # Search-stage coarse alignment guard.
    # 8 deg以内：够用于找包裹，不转。
    # 12 deg以上：才开始粗对正。
    search_already_ok_deg: 8.0
    search_start_align_deg: 12.0
    same_shelf_skip_sec: 12.0
    search_align_only: true
"""

    if s != old:
        patch_file_text(p, s, "patched OAK yaml guard")
    else:
        print(f"[NO CHANGE] OAK yaml: {p}")

# ------------------------------------------------------------
# 3) Patch mission_btcpp_node:
#    - if already near target approach pose, do not publish /goal_pose again
# ------------------------------------------------------------

def patch_mission_btcpp_node(p: Path):
    s = p.read_text(errors="ignore")
    old = s

    if "already near target approach pose; skip /goal_pose" in s:
        print(f"[SKIP] mission nav re-entry guard already exists: {p}")
        return

    marker = "geometry_msgs::msg::PoseStamped goal;goal.header.stamp=ctx->node->now();"

    guard = r'''try{
    auto tf=ctx->tf_buffer->lookupTransform(ctx->map_frame,ctx->base_frame,tf2::TimePointZero,tf2::durationFromSec(0.20));
    double dx=tf.transform.translation.x-ctx->resolved_pose.x;
    double dy=tf.transform.translation.y-ctx->resolved_pose.y;
    double dist=std::hypot(dx,dy);
    if(dist<0.45){
        publish_stop(ctx);
        ctx->control_mode_pub->publish(std_msgs::msg::String().set__data("stop"));
        std::stringstream ss;
        ss<<"already near target approach pose; skip /goal_pose dist="<<dist;
        ctx->status("SUCCESS",ss.str());
        ctx->trace(name(),"SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
}catch(const std::exception& e){
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(),*ctx->node->get_clock(),1000,
        "[NavigateToShelf] re-entry distance check failed: %s",e.what());
}
'''

    if marker in s:
        s = s.replace(marker, guard + marker, 1)
    else:
        print(f"[WARN] mission NavigateToShelf marker not found: {p}")
        print("       Navigation re-entry guard was not inserted.")
        return

    if s != old:
        patch_file_text(p, s, "patched mission navigation re-entry guard")

# ------------------------------------------------------------
# 4) Patch Nav2 params:
#    - do not force final goal heading rotation
# ------------------------------------------------------------

def patch_nav2_yaml(p: Path):
    s = p.read_text(errors="ignore")
    old = s

    if "yaw_goal_tolerance" in s:
        s = re.sub(r"yaw_goal_tolerance:\s*[-0-9.]+", "yaw_goal_tolerance: 3.14", s)

    if "xy_goal_tolerance" in s:
        s = re.sub(r"xy_goal_tolerance:\s*0\.18\b", "xy_goal_tolerance: 0.35", s)
        s = re.sub(r"xy_goal_tolerance:\s*0\.22\b", "xy_goal_tolerance: 0.35", s)
        s = re.sub(r"xy_goal_tolerance:\s*0\.25\b", "xy_goal_tolerance: 0.35", s)
        s = re.sub(r"xy_goal_tolerance:\s*0\.30\b", "xy_goal_tolerance: 0.35", s)

    if "rotate_to_goal_heading" in s:
        s = re.sub(r"rotate_to_goal_heading:\s*true", "rotate_to_goal_heading: false", s, flags=re.I)

    if "RotateToGoal" in s:
        s = re.sub(r"scale:\s*[-0-9.]+", "scale: 0.0", s)

    if s != old:
        patch_file_text(p, s, "patched Nav2 no-final-hard-rotate")
    else:
        print(f"[NO CHANGE] Nav2 yaml: {p}")

# ------------------------------------------------------------
# Run patches
# ------------------------------------------------------------

oak_py_files = list(SRC_ROOT.glob("**/oak_package_search_node.py"))
oak_py_files += list(INSTALL_ROOT.glob("**/oak_package_search_node.py"))

if not oak_py_files:
    print("[WARN] oak_package_search_node.py not found")

for p in oak_py_files:
    patch_oak_package_search_node(p)

oak_yaml_files = list(SRC_ROOT.glob("**/oak_package_search*.yaml"))
oak_yaml_files += list(INSTALL_ROOT.glob("**/oak_package_search*.yaml"))

for p in oak_yaml_files:
    patch_oak_yaml(p)

mission_files = list(SRC_ROOT.glob("**/robot_logistics_task/src/mission_btcpp_node.cpp"))
if not mission_files:
    print("[WARN] mission_btcpp_node.cpp not found")

for p in mission_files:
    patch_mission_btcpp_node(p)

nav2_yaml_files = []
for root in [SRC_ROOT / "Navigation", INSTALL_ROOT]:
    if root.exists():
        nav2_yaml_files += list(root.glob("**/nav2*.yaml"))
        nav2_yaml_files += list(root.glob("**/*nav2*params*.yaml"))

seen = set()
for p in nav2_yaml_files:
    if p in seen:
        continue
    seen.add(p)
    patch_nav2_yaml(p)

print("")
print("[DONE] Patch finished.")
print("Next: rebuild robot_logistics_task and restart bringup.")
