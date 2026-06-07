from pathlib import Path
import re
import time
import ast

p = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py")

if not p.exists():
    raise SystemExit(f"[ERROR] file not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
s = p.read_text(errors="ignore")
bak = p.with_suffix(p.suffix + f".bak_tf_motion_drift_filter_{stamp}")
bak.write_text(s)
print(f"[BACKUP] {p} -> {bak}")

# ------------------------------------------------------------
# 1. Ensure TF motion parameters exist and tune defaults
# ------------------------------------------------------------

def replace_declare_float(text, name, value):
    pattern = rf"(declare_parameter\(\s*['\"]{re.escape(name)}['\"]\s*,\s*)([-+]?\d+(\.\d+)?)(\s*\))"
    text2 = re.sub(pattern, rf"\g<1>{value}\g<4>", text, count=1)
    return text2

def replace_declare_bool(text, name, value):
    val = "True" if value else "False"
    pattern = rf"(declare_parameter\(\s*['\"]{re.escape(name)}['\"]\s*,\s*)(True|False)(\s*\))"
    text2 = re.sub(pattern, rf"\g<1>{val}\g<3>", text, count=1)
    return text2

# Update existing default values if already present.
for name, value in [
    ("motion_tf_linear_delta_threshold_m", 0.05),
    ("motion_tf_yaw_delta_threshold_deg", 5.0),
    ("motion_tf_linear_speed_threshold_mps", 0.08),
    ("motion_tf_angular_speed_threshold_radps", 0.15),
    ("motion_tf_settle_time_sec", 1.0),
    ("motion_tf_lookup_timeout_sec", 0.05),
]:
    s = replace_declare_float(s, name, value)

s = replace_declare_bool(s, "freeze_update_when_robot_tf_moving", True)

# Add new anti-drift parameters if missing.
if "motion_tf_sample_min_dt_sec" not in s:
    anchor = "        self.declare_parameter('motion_tf_lookup_timeout_sec', 0.05)\n"
    if anchor not in s:
        # If the exact old line was already changed, insert after motion_tf_settle_time_sec.
        anchor = "        self.declare_parameter('motion_tf_settle_time_sec', 1.0)\n"

    if anchor not in s:
        raise SystemExit("[ERROR] cannot find place to insert anti-drift TF params")

    insert = """        # Anti-drift filter for RF2O / map TF small jitter.
        # Do not treat tiny pose noise as real robot motion.
        self.declare_parameter('motion_tf_sample_min_dt_sec', 0.35)
        self.declare_parameter('motion_tf_position_deadband_m', 0.015)
        self.declare_parameter('motion_tf_yaw_deadband_deg', 1.0)
        self.declare_parameter('motion_tf_confirm_count', 2)
        self.declare_parameter('motion_tf_release_count', 3)
        self.declare_parameter('motion_tf_baseline_reset_sec', 3.0)
"""
    s = s.replace(anchor, anchor + insert, 1)
    print("[OK] inserted anti-drift TF parameters")
else:
    print("[SKIP] anti-drift TF parameters already exist")

# ------------------------------------------------------------
# 2. Ensure state variables exist
# ------------------------------------------------------------

if "self.motion_tf_baseline_poses" not in s:
    anchor = "        self.motion_tf_last_poses = {}\n"
    if anchor not in s:
        raise SystemExit("[ERROR] cannot find self.motion_tf_last_poses init")

    insert = """        self.motion_tf_baseline_poses = {}
        self.motion_tf_moving_votes = {}
        self.motion_tf_still_votes = {}
        self.motion_tf_current_moving = False
"""
    s = s.replace(anchor, anchor + insert, 1)
    print("[OK] added anti-drift TF state variables")
else:
    print("[SKIP] anti-drift TF state variables already exist")

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

        # Try configured robot_frame first.
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

        # Anything inside this area is treated as sensor / odom jitter.
        eff_dist = max(0.0, float(dist) - pos_deadband)
        eff_yaw = max(0.0, float(yaw_abs) - yaw_deadband)
        return eff_dist, eff_yaw

    def update_motion_from_tf_reference(self, reference_frame):
        """
        Return True only when TF movement is large and persistent enough.

        Design:
          - last pose is used for instant speed estimation.
          - baseline pose is used for accumulated displacement.
          - small odom/map jitter is ignored by deadband.
          - motion must be confirmed by N consecutive samples.
          - stillness must be confirmed before releasing freeze.
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
                # Too soon. Do not calculate speed from tiny dt, because it amplifies jitter.
                return bool(self.motion_tf_current_moving)

        self.motion_tf_last_poses[key] = pose

        baseline = self.motion_tf_baseline_poses.get(key)
        if baseline is None:
            self.motion_tf_baseline_poses[key] = pose
            self.motion_tf_moving_votes[key] = 0
            self.motion_tf_still_votes[key] = 0
            return False

        # Accumulated change since the last stable baseline.
        dist_total, yaw_total = self.tf_pose_delta(pose, baseline)
        eff_dist_total, eff_yaw_total = self.apply_tf_deadband(dist_total, yaw_total)

        # Instant speed since last accepted sample.
        linear_speed = 0.0
        angular_speed = 0.0
        if last is not None:
            dt = max(now - float(last.get('t', now)), 1e-3)
            dist_step, yaw_step = self.tf_pose_delta(pose, last)
            eff_dist_step, eff_yaw_step = self.apply_tf_deadband(dist_step, yaw_step)
            linear_speed = eff_dist_step / dt
            angular_speed = eff_yaw_step / dt

        delta_th = float(self.get_parameter('motion_tf_linear_delta_threshold_m').value)
        yaw_delta_th = math.radians(
            float(self.get_parameter('motion_tf_yaw_delta_threshold_deg').value)
        )
        linear_speed_th = float(self.get_parameter('motion_tf_linear_speed_threshold_mps').value)
        angular_speed_th = float(self.get_parameter('motion_tf_angular_speed_threshold_radps').value)

        # Real movement can be detected by accumulated change OR by speed.
        # Small fixed-range drifting should stay below both after deadband.
        candidate_moving = (
            eff_dist_total > delta_th or
            eff_yaw_total > yaw_delta_th or
            linear_speed > linear_speed_th or
            angular_speed > angular_speed_th
        )

        confirm_count = int(self.get_parameter('motion_tf_confirm_count').value)
        release_count = int(self.get_parameter('motion_tf_release_count').value)
        baseline_reset_sec = float(self.get_parameter('motion_tf_baseline_reset_sec').value)

        if candidate_moving:
            self.motion_tf_moving_votes[key] = int(self.motion_tf_moving_votes.get(key, 0)) + 1
            self.motion_tf_still_votes[key] = 0

            if self.motion_tf_moving_votes[key] >= max(1, confirm_count):
                self.get_logger().info(
                    f'TF real motion confirmed ref={pose["reference_frame"]}, base={pose["base_frame"]}, '
                    f'acc_dist={dist_total:.3f}m, acc_yaw={math.degrees(yaw_total):.1f}deg, '
                    f'eff_dist={eff_dist_total:.3f}m, eff_yaw={math.degrees(eff_yaw_total):.1f}deg, '
                    f'v={linear_speed:.3f}m/s, wz={angular_speed:.3f}rad/s',
                    throttle_duration_sec=1.0
                )
                self.motion_tf_last_motion_time = now
                self.motion_tf_current_moving = True
                return True

            return bool(self.motion_tf_current_moving)

        # Not moving candidate.
        self.motion_tf_moving_votes[key] = 0
        self.motion_tf_still_votes[key] = int(self.motion_tf_still_votes.get(key, 0)) + 1

        settle_time = float(self.get_parameter('motion_tf_settle_time_sec').value)

        # Reset baseline when the robot has been still for a while.
        # This prevents tiny odom drift from accumulating forever.
        baseline_age = now - float(baseline.get('t', now))
        if baseline_age > baseline_reset_sec and not self.motion_tf_current_moving:
            self.motion_tf_baseline_poses[key] = pose

        if self.motion_tf_current_moving:
            enough_still_samples = self.motion_tf_still_votes[key] >= max(1, release_count)
            enough_settle_time = (now - float(self.motion_tf_last_motion_time)) >= settle_time

            if enough_still_samples and enough_settle_time:
                self.motion_tf_current_moving = False
                self.motion_tf_baseline_poses[key] = pose
                self.get_logger().info(
                    f'TF motion freeze released ref={pose["reference_frame"]}, base={pose["base_frame"]}',
                    throttle_duration_sec=1.0
                )

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
print("[OK] replaced TF motion helper logic with anti-drift filter")

# ------------------------------------------------------------
# 4. Syntax check
# ------------------------------------------------------------
ast.parse(s)
p.write_text(s)

print("")
print("[DONE]")
print("TF motion gate now ignores small fixed-range odom/map drift.")
print("Recommended defaults:")
print("  position deadband: 0.015 m")
print("  yaw deadband: 1.0 deg")
print("  movement threshold: 0.05 m / 5 deg")
print("  speed threshold: 0.08 m/s / 0.15 rad/s")
print("  confirm count: 2 samples")
