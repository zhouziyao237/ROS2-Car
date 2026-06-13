from pathlib import Path
import ast
import re
import time

p = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py")

if not p.exists():
    raise SystemExit(f"[ERROR] file not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
s = p.read_text(errors="ignore")
bak = p.with_suffix(p.suffix + f".bak_tf_sliding_window_{stamp}")
bak.write_text(s)
print(f"[BACKUP] {p} -> {bak}")

# ------------------------------------------------------------
# 1. Add / update parameters
# ------------------------------------------------------------
def replace_declare(text, name, value):
    pattern = rf"(declare_parameter\(\s*['\"]{re.escape(name)}['\"]\s*,\s*)([-+]?\d+(\.\d+)?)(\s*\))"
    text2 = re.sub(pattern, rf"\g<1>{value}\g<4>", text, count=1)
    return text2

# These are now used for short-window motion, not infinite accumulated baseline.
defaults = {
    "motion_tf_position_deadband_m": 0.004,
    "motion_tf_yaw_deadband_deg": 0.2,

    "motion_tf_linear_delta_threshold_m": 0.015,
    "motion_tf_yaw_delta_threshold_deg": 0.8,

    "motion_tf_linear_speed_threshold_mps": 0.015,
    "motion_tf_angular_speed_threshold_radps": 0.025,

    "motion_tf_confirm_count": 1,
    "motion_tf_release_count": 5,
    "motion_tf_settle_time_sec": 2.0,
    "motion_tf_sample_min_dt_sec": 0.25,
}

for k, v in defaults.items():
    s = replace_declare(s, k, v)

if "motion_tf_window_sec" not in s:
    anchor = "        self.declare_parameter('motion_tf_baseline_reset_sec',"
    idx = s.find(anchor)
    if idx >= 0:
        line_end = s.find("\n", idx)
        insert = """
        # Sliding-window motion detection.
        # Use recent TF motion only; do not accumulate odom/map drift forever.
        self.declare_parameter('motion_tf_window_sec', 1.2)
"""
        s = s[:line_end+1] + insert + s[line_end+1:]
        print("[OK] added motion_tf_window_sec")
    else:
        anchor2 = "        self.declare_parameter('motion_tf_sample_min_dt_sec',"
        idx = s.find(anchor2)
        if idx < 0:
            raise SystemExit("[ERROR] cannot find TF motion parameter block")
        line_end = s.find("\n", idx)
        insert = """
        self.declare_parameter('motion_tf_window_sec', 1.2)
"""
        s = s[:line_end+1] + insert + s[line_end+1:]
        print("[OK] added motion_tf_window_sec")
else:
    s = replace_declare(s, "motion_tf_window_sec", 1.2)
    print("[OK] updated motion_tf_window_sec")

# ------------------------------------------------------------
# 2. Ensure state variables
# ------------------------------------------------------------
if "self.motion_tf_pose_windows" not in s:
    anchor = "        self.motion_tf_last_poses = {}\n"
    if anchor not in s:
        raise SystemExit("[ERROR] cannot find self.motion_tf_last_poses init")

    insert = """        self.motion_tf_pose_windows = {}
        self.motion_tf_moving_votes = {}
        self.motion_tf_still_votes = {}
        self.motion_tf_current_moving = False
"""
    # Avoid duplicate old state block if it already has some of these names
    if "self.motion_tf_current_moving" in s:
        insert = """        self.motion_tf_pose_windows = {}
"""
    s = s.replace(anchor, anchor + insert, 1)
    print("[OK] added sliding-window TF state")
else:
    print("[SKIP] sliding-window TF state already exists")

# ------------------------------------------------------------
# 3. Replace helper methods from now_sec() to before transform_pose_to_map()
# ------------------------------------------------------------
start = s.find("    def now_sec(self):")
end = s.find("    def transform_pose_to_map(self, pose_stamped):")

if start < 0 or end < 0 or end <= start:
    raise SystemExit("[ERROR] cannot find helper method region: now_sec -> transform_pose_to_map")

helper = r'''    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def get_string_list_parameter(self, name, fallback):
        try:
            value = self.get_parameter(name).value
            if value is None:
                return list(fallback)
            if isinstance(value, (list, tuple)):
                return [str(x) for x in value]
            if isinstance(value, str):
                return [x.strip() for x in value.split(',') if x.strip()]
            return list(fallback)
        except Exception:
            return list(fallback)

    def lookup_robot_motion_pose(self, reference_frame):
        base_candidates = self.get_string_list_parameter(
            'motion_tf_base_frame_candidates',
            ['base_link', 'base_footprint']
        )

        robot_frame = str(self.robot_frame)
        if robot_frame and robot_frame not in base_candidates:
            base_candidates = [robot_frame] + base_candidates

        timeout = float(self.get_parameter('motion_tf_lookup_timeout_sec').value)

        for base_frame in base_candidates:
            try:
                tf = self.tf_buffer.lookup_transform(
                    str(reference_frame),
                    str(base_frame),
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=timeout),
                )

                q = tf.transform.rotation
                yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

                return {
                    'reference_frame': str(reference_frame),
                    'base_frame': str(base_frame),
                    'x': float(tf.transform.translation.x),
                    'y': float(tf.transform.translation.y),
                    'yaw': float(yaw),
                    't': self.now_sec(),
                }
            except Exception:
                continue

        return None

    def tf_pose_delta(self, a, b):
        dx = float(a['x']) - float(b['x'])
        dy = float(a['y']) - float(b['y'])
        dyaw = self.normalize_angle(float(a['yaw']) - float(b['yaw']))
        return math.hypot(dx, dy), abs(dyaw)

    def apply_tf_deadband(self, dist, yaw_abs):
        pos_deadband = float(self.get_parameter('motion_tf_position_deadband_m').value)
        yaw_deadband = math.radians(
            float(self.get_parameter('motion_tf_yaw_deadband_deg').value)
        )

        eff_dist = max(0.0, float(dist) - pos_deadband)
        eff_yaw = max(0.0, float(yaw_abs) - yaw_deadband)
        return eff_dist, eff_yaw

    def update_motion_from_tf_reference(self, reference_frame):
        """
        Sliding-window TF motion detector.

        Important:
          - It does NOT use infinite accumulated odom/map drift.
          - It only checks recent movement inside motion_tf_window_sec.
          - This avoids false freezing when the robot is static but odom slowly drifts.
          - It still detects slow pushing because recent window delta / speed changes.
        """
        pose = self.lookup_robot_motion_pose(reference_frame)
        if pose is None:
            return False

        key = f"{pose['reference_frame']}->{pose['base_frame']}"
        now = float(pose['t'])

        sample_min_dt = float(self.get_parameter('motion_tf_sample_min_dt_sec').value)

        last = self.motion_tf_last_poses.get(key)
        if last is not None:
            dt_last = now - float(last.get('t', now))
            if dt_last < sample_min_dt:
                return bool(self.motion_tf_current_moving)

        self.motion_tf_last_poses[key] = pose

        if not hasattr(self, 'motion_tf_pose_windows'):
            self.motion_tf_pose_windows = {}

        window = self.motion_tf_pose_windows.get(key, [])
        window.append(pose)

        window_sec = float(self.get_parameter('motion_tf_window_sec').value)
        window = [x for x in window if now - float(x.get('t', now)) <= window_sec]

        # Keep at least one pose.
        if not window:
            window = [pose]

        self.motion_tf_pose_windows[key] = window

        if len(window) < 2:
            self.motion_tf_moving_votes[key] = 0
            self.motion_tf_still_votes[key] = int(self.motion_tf_still_votes.get(key, 0)) + 1
            return bool(self.motion_tf_current_moving)

        oldest = window[0]

        # Recent window delta.
        win_dist, win_yaw = self.tf_pose_delta(pose, oldest)
        eff_win_dist, eff_win_yaw = self.apply_tf_deadband(win_dist, win_yaw)

        win_dt = max(now - float(oldest.get('t', now)), 1e-3)
        win_linear_speed = eff_win_dist / win_dt
        win_angular_speed = eff_win_yaw / win_dt

        # Step delta from previous accepted sample.
        step_dist = 0.0
        step_yaw = 0.0
        step_linear_speed = 0.0
        step_angular_speed = 0.0

        if len(window) >= 2:
            prev = window[-2]
            dt = max(now - float(prev.get('t', now)), 1e-3)
            step_dist, step_yaw = self.tf_pose_delta(pose, prev)
            eff_step_dist, eff_step_yaw = self.apply_tf_deadband(step_dist, step_yaw)
            step_linear_speed = eff_step_dist / dt
            step_angular_speed = eff_step_yaw / dt

        delta_th = float(self.get_parameter('motion_tf_linear_delta_threshold_m').value)
        yaw_delta_th = math.radians(
            float(self.get_parameter('motion_tf_yaw_delta_threshold_deg').value)
        )
        linear_speed_th = float(self.get_parameter('motion_tf_linear_speed_threshold_mps').value)
        angular_speed_th = float(self.get_parameter('motion_tf_angular_speed_threshold_radps').value)

        candidate_moving = (
            eff_win_dist > delta_th or
            eff_win_yaw > yaw_delta_th or
            win_linear_speed > linear_speed_th or
            win_angular_speed > angular_speed_th or
            step_linear_speed > linear_speed_th or
            step_angular_speed > angular_speed_th
        )

        confirm_count = int(self.get_parameter('motion_tf_confirm_count').value)
        release_count = int(self.get_parameter('motion_tf_release_count').value)

        if candidate_moving:
            self.motion_tf_moving_votes[key] = int(self.motion_tf_moving_votes.get(key, 0)) + 1
            self.motion_tf_still_votes[key] = 0

            if self.motion_tf_moving_votes[key] >= max(1, confirm_count):
                self.motion_tf_current_moving = True
                self.motion_tf_last_motion_time = now
                self.get_logger().info(
                    f'TF window motion confirmed ref={pose["reference_frame"]}, base={pose["base_frame"]}, '
                    f'win_dist={win_dist:.3f}m, win_yaw={math.degrees(win_yaw):.1f}deg, '
                    f'eff_win_dist={eff_win_dist:.3f}m, eff_win_yaw={math.degrees(eff_win_yaw):.1f}deg, '
                    f'win_v={win_linear_speed:.3f}m/s, win_wz={win_angular_speed:.3f}rad/s, '
                    f'step_v={step_linear_speed:.3f}m/s, step_wz={step_angular_speed:.3f}rad/s',
                    throttle_duration_sec=1.0
                )
                return True

            return bool(self.motion_tf_current_moving)

        # No movement candidate in recent window.
        self.motion_tf_moving_votes[key] = 0
        self.motion_tf_still_votes[key] = int(self.motion_tf_still_votes.get(key, 0)) + 1

        if self.motion_tf_current_moving:
            settle_time = float(self.get_parameter('motion_tf_settle_time_sec').value)
            enough_still_samples = self.motion_tf_still_votes[key] >= max(1, release_count)
            enough_settle_time = (now - float(self.motion_tf_last_motion_time)) >= settle_time

            if enough_still_samples and enough_settle_time:
                self.motion_tf_current_moving = False
                # Reset windows so old motion cannot keep freezing.
                self.motion_tf_pose_windows = {}
                self.motion_tf_last_poses = {}
                self.motion_tf_moving_votes = {}
                self.motion_tf_still_votes = {}
                self.get_logger().info(
                    'TF motion freeze released; sliding windows reset',
                    throttle_duration_sec=1.0
                )
                return False

        return bool(self.motion_tf_current_moving)

    def is_robot_moving_by_tf_for_semantic_update(self):
        if not bool(self.get_parameter('freeze_update_when_robot_tf_moving').value):
            return False

        reference_frames = self.get_string_list_parameter(
            'motion_tf_reference_frames',
            ['odom', 'map']
        )

        moving = False
        for ref in reference_frames:
            if self.update_motion_from_tf_reference(ref):
                moving = True

        return moving

'''

s = s[:start] + helper + s[end:]

# ------------------------------------------------------------
# 4. Syntax check
# ------------------------------------------------------------
ast.parse(s)
p.write_text(s)

print("")
print("[DONE]")
print("Replaced infinite accumulated TF baseline with sliding-window motion detection.")
print("This should stop false freezing when the robot is static but odom/map slowly drifts.")
