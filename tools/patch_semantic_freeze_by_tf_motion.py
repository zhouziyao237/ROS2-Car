from pathlib import Path
import ast
import re
import time

p = Path("/host-home/src/Navigation/robot_logistics_semantic/robot_logistics_semantic/semantic_registrar_node.py")

if not p.exists():
    raise SystemExit(f"[ERROR] file not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
s = p.read_text(errors="ignore")
bak = p.with_suffix(p.suffix + f".bak_tf_motion_freeze_{stamp}")
bak.write_text(s)
print(f"[BACKUP] {p} -> {bak}")

# ------------------------------------------------------------
# 1. Add parameters
# ------------------------------------------------------------
if "freeze_update_when_robot_tf_moving" not in s:
    anchor = "        self.declare_parameter('tf_timeout_sec', 0.5)\n"
    if anchor not in s:
        raise SystemExit("[ERROR] cannot find tf_timeout_sec parameter block")

    insert = """
        # Freeze semantic shelf pose update while robot is moving.
        # This uses TF pose change, so it also works when the robot is pushed by hand.
        self.declare_parameter('freeze_update_when_robot_tf_moving', True)
        self.declare_parameter('motion_tf_reference_frames', ['odom', 'map'])
        self.declare_parameter('motion_tf_base_frame_candidates', ['base_link', 'base_footprint'])
        self.declare_parameter('motion_tf_linear_delta_threshold_m', 0.02)
        self.declare_parameter('motion_tf_yaw_delta_threshold_deg', 2.0)
        self.declare_parameter('motion_tf_linear_speed_threshold_mps', 0.02)
        self.declare_parameter('motion_tf_angular_speed_threshold_radps', 0.03)
        self.declare_parameter('motion_tf_settle_time_sec', 1.0)
        self.declare_parameter('motion_tf_lookup_timeout_sec', 0.05)
"""
    s = s.replace(anchor, anchor + insert, 1)
    print("[OK] added TF motion freeze parameters")
else:
    print("[SKIP] TF motion freeze parameters already exist")

# ------------------------------------------------------------
# 2. Add state variables after tf listener
# ------------------------------------------------------------
if "self.motion_tf_last_poses" not in s:
    anchor = """        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
"""
    if anchor not in s:
        raise SystemExit("[ERROR] cannot find tf_buffer/tf_listener block")

    insert = """
        self.motion_tf_last_poses = {}
        self.motion_tf_last_motion_time = 0.0
"""
    s = s.replace(anchor, anchor + insert, 1)
    print("[OK] added TF motion state variables")
else:
    print("[SKIP] TF motion state variables already exist")

# ------------------------------------------------------------
# 3. Insert gate inside obs_cb after confidence check, before transform_pose_to_map
# ------------------------------------------------------------
gate = """        if self.is_robot_moving_by_tf_for_semantic_update():
            self.get_logger().info(
                f'skip semantic shelf update while robot moving by TF: '
                f'obs={obs.obs_type} shelf={obs.shelf_id} face={obs.face}',
                throttle_duration_sec=1.0
            )
            return

"""

if "skip semantic shelf update while robot moving by TF" not in s:
    anchor = """        if float(obs.confidence) < self.min_confidence:
            self.get_logger().warn(
                f'ignore {obs.obs_type}: confidence={float(obs.confidence):.2f} too low'
            )
            return

"""
    if anchor not in s:
        raise SystemExit("[ERROR] cannot find confidence check block in obs_cb")

    s = s.replace(anchor, anchor + gate, 1)
    print("[OK] inserted TF motion freeze gate in obs_cb")
else:
    print("[SKIP] TF motion freeze gate already exists")

# ------------------------------------------------------------
# 4. Add helper methods before transform_pose_to_map
# ------------------------------------------------------------
if "def is_robot_moving_by_tf_for_semantic_update(self):" not in s:
    anchor = "    def transform_pose_to_map(self, pose_stamped):\n"
    if anchor not in s:
        raise SystemExit("[ERROR] cannot find transform_pose_to_map")

    helper = '''    def now_sec(self):
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

        # Also try the configured robot_frame first.
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

    def update_motion_from_tf_reference(self, reference_frame):
        pose = self.lookup_robot_motion_pose(reference_frame)
        if pose is None:
            return False

        key = pose['reference_frame']
        now = pose['t']

        last = self.motion_tf_last_poses.get(key)
        self.motion_tf_last_poses[key] = pose

        if last is None:
            return False

        dt = max(now - float(last.get('t', now)), 1e-3)

        dx = pose['x'] - float(last.get('x', pose['x']))
        dy = pose['y'] - float(last.get('y', pose['y']))
        dyaw = self.normalize_angle(pose['yaw'] - float(last.get('yaw', pose['yaw'])))

        dist = math.hypot(dx, dy)
        yaw_abs = abs(dyaw)

        linear_speed = dist / dt
        angular_speed = yaw_abs / dt

        delta_th = float(self.get_parameter('motion_tf_linear_delta_threshold_m').value)
        yaw_delta_th = math.radians(
            float(self.get_parameter('motion_tf_yaw_delta_threshold_deg').value)
        )
        linear_speed_th = float(self.get_parameter('motion_tf_linear_speed_threshold_mps').value)
        angular_speed_th = float(self.get_parameter('motion_tf_angular_speed_threshold_radps').value)

        moving = (
            dist > delta_th or
            yaw_abs > yaw_delta_th or
            linear_speed > linear_speed_th or
            angular_speed > angular_speed_th
        )

        if moving:
            self.get_logger().info(
                f'TF motion detected ref={key}, base={pose["base_frame"]}, '
                f'dist={dist:.3f}m, dyaw={math.degrees(yaw_abs):.1f}deg, '
                f'v={linear_speed:.3f}m/s, wz={angular_speed:.3f}rad/s',
                throttle_duration_sec=1.0
            )

        return moving

    def is_robot_moving_by_tf_for_semantic_update(self):
        if not bool(self.get_parameter('freeze_update_when_robot_tf_moving').value):
            return False

        reference_frames = self.get_string_list_parameter(
            'motion_tf_reference_frames',
            ['odom', 'map']
        )

        now = self.now_sec()

        moving_now = False
        for ref in reference_frames:
            if self.update_motion_from_tf_reference(ref):
                moving_now = True

        if moving_now:
            self.motion_tf_last_motion_time = now
            return True

        settle_time = float(self.get_parameter('motion_tf_settle_time_sec').value)
        if self.motion_tf_last_motion_time > 0.0:
            if now - self.motion_tf_last_motion_time < settle_time:
                return True

        return False

'''
    s = s.replace(anchor, helper + anchor, 1)
    print("[OK] added TF motion helper methods")
else:
    print("[SKIP] TF motion helper methods already exist")

# ------------------------------------------------------------
# 5. Syntax check
# ------------------------------------------------------------
ast.parse(s)
p.write_text(s)

print("")
print("[DONE]")
print("semantic_registrar_node will now freeze shelf pose update when odom/map TF shows robot motion.")
print("It can detect manual pushing because it checks actual TF pose change, not only /cmd_vel.")
