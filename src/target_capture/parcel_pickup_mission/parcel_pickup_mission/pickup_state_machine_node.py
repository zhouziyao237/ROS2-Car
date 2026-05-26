#!/usr/bin/env python3
import json
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import PointStamped, Twist

import tf2_ros
from rclpy.time import Time


class MissionState(str, Enum):
    IDLE = "IDLE"
    PARSE_PICKUP_CODE = "PARSE_PICKUP_CODE"

    RECORD_CURRENT_ANCHOR = "RECORD_CURRENT_ANCHOR"

    SET_COL_TARGET = "SET_COL_TARGET"
    VERIFY_COL_AT_ANCHOR = "VERIFY_COL_AT_ANCHOR"

    SET_ROW_TARGET = "SET_ROW_TARGET"
    VERIFY_ROW_AT_ANCHOR = "VERIFY_ROW_AT_ANCHOR"

    COARSE_LIFT_TO_ROW = "COARSE_LIFT_TO_ROW"
    WAIT_ROW_HEIGHT = "WAIT_ROW_HEIGHT"

    SET_PACKAGE_SEARCH_TARGET = "SET_PACKAGE_SEARCH_TARGET"
    SEARCH_PACKAGE_AT_ANCHOR = "SEARCH_PACKAGE_AT_ANCHOR"

    SET_PICK_PACKAGE_TARGET = "SET_PICK_PACKAGE_TARGET"
    WAIT_OAK_VIEW = "WAIT_OAK_VIEW"

    CHASSIS_FINE_ALIGN = "CHASSIS_FINE_ALIGN"
    LIFT_FINE_ALIGN = "LIFT_FINE_ALIGN"
    FREEZE_LIFT = "FREEZE_LIFT"

    EXTEND_PLATFORM_AND_HAND = "EXTEND_PLATFORM_AND_HAND"
    WAIT_EXTENDED = "WAIT_EXTENDED"

    RETRACT_HAND_AND_PLATFORM = "RETRACT_HAND_AND_PLATFORM"
    WAIT_RETRACTED = "WAIT_RETRACTED"

    DONE = "DONE"
    ERROR = "ERROR"


class ParcelPickupMissionNode(Node):
    def __init__(self):
        super().__init__("parcel_pickup_mission_node")

        # =========================
        # Mode
        # =========================
        self.declare_parameter("camera_mode", "usb_wide_oak_pick")
        self.declare_parameter("mirror_pick_target_during_search", True)

        # =========================
        # TF
        # =========================
        self.declare_parameter("nav_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("require_tf_anchor", False)

        # =========================
        # Topics
        # =========================
        self.declare_parameter("pickup_code_topic", "/mission/pickup_code")
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("mission_debug_topic", "/mission/debug")

        self.declare_parameter("wide_target_code_topic", "/wide_target_code")
        self.declare_parameter("wide_qr_state_topic", "/wide_qr/state")
        self.declare_parameter("wide_qr_raw_topic", "/wide_qr/raw")

        self.declare_parameter("pick_target_code_topic", "/target_code")
        self.declare_parameter("pick_qr_state_topic", "/target_matcher/state")
        self.declare_parameter("pick_qr_raw_topic", "/target_matcher/raw")
        self.declare_parameter("pick_target_pose_topic", "/target_pose_base")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_selected")
        self.declare_parameter("set_lift_topic", "/qr_lift_control/set_lift_mm")
        self.declare_parameter("chassis_align_enable_topic", "/qr_chassis_align/enable")
        self.declare_parameter("lift_enable_topic", "/qr_lift_control/enable")
        self.declare_parameter("lift_freeze_topic", "/qr_lift_control/freeze")
        self.declare_parameter("pickup_extend_start_topic", "/pickup_extend/start")
        self.declare_parameter("pickup_extend_retract_topic", "/pickup_extend/retract")

        # =========================
        # QR format
        # =========================
        self.declare_parameter("shelf_col_prefix", "COL:")
        self.declare_parameter("shelf_row_prefix", "ROW:")

        # =========================
        # Verify timeout
        # =========================
        self.declare_parameter("require_col_verify", True)
        self.declare_parameter("col_verify_timeout_sec", 8.0)
        self.declare_parameter("row_verify_timeout_sec", 10.0)
        self.declare_parameter("package_search_timeout_sec", 15.0)
        self.declare_parameter("oak_view_timeout_sec", 10.0)

        # =========================
        # Row height
        # =========================
        self.declare_parameter("row_ids", ["1", "2", "3", "4"])
        self.declare_parameter("row_heights_mm", [0.0, 20.0, 40.0, 60.0])
        self.declare_parameter("base_height_mm", 0.0)
        self.declare_parameter("level_gap_mm", 20.0)
        self.declare_parameter("row_height_wait_sec", 2.0)

        # =========================
        # Package search lift scan
        # =========================
        self.declare_parameter("package_search_enable_lift_scan", True)
        self.declare_parameter("package_search_range_mm", 25.0)
        self.declare_parameter("package_search_step_mm", 5.0)
        self.declare_parameter("package_search_interval_sec", 0.70)
        self.declare_parameter("lift_min_mm", 0.0)
        self.declare_parameter("lift_max_mm", 350.0)

        # =========================
        # Fine alignment
        # /target_pose_base:
        # x = 左右误差
        # y = 升降误差
        # z = 深度
        # =========================
        self.declare_parameter("x_target_mm", 0.0)
        self.declare_parameter("y_target_mm", 0.0)
        self.declare_parameter("x_deadband_mm", 35.0)
        self.declare_parameter("y_deadband_mm", 30.0)
        self.declare_parameter("stable_required_frames", 5)
        self.declare_parameter("pose_timeout_sec", 0.8)

        self.declare_parameter("chassis_align_timeout_sec", 10.0)
        self.declare_parameter("lift_align_timeout_sec", 15.0)

        self.declare_parameter("extend_wait_sec", 4.0)
        self.declare_parameter("retract_wait_sec", 3.0)
        self.declare_parameter("state_period_sec", 0.10)

        # =========================
        # Runtime
        # =========================
        self.state = MissionState.IDLE
        self.state_enter_time = self.now_sec()

        self.pickup_code = ""
        self.col = 0
        self.row = 0
        self.package_code = ""
        self.row_target_height_mm = 0.0

        self.wide_state = ""
        self.wide_raw = ""

        self.pick_state = ""
        self.pick_raw = ""

        self.pose = None
        self.pose_time = None

        self.x_stable_count = 0
        self.y_stable_count = 0

        self.col_anchor = None

        self.search_lift_center_mm = 0.0
        self.search_lift_cmd_mm = 0.0
        self.search_lift_dir = 1
        self.last_search_lift_time = 0.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =========================
        # Publishers
        # =========================
        self.state_pub = self.create_publisher(
            String,
            self.get_parameter("mission_state_topic").value,
            10,
        )

        self.debug_pub = self.create_publisher(
            String,
            self.get_parameter("mission_debug_topic").value,
            10,
        )

        self.wide_target_pub = self.create_publisher(
            String,
            self.get_parameter("wide_target_code_topic").value,
            10,
        )

        self.pick_target_pub = self.create_publisher(
            String,
            self.get_parameter("pick_target_code_topic").value,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        self.set_lift_pub = self.create_publisher(
            Float32,
            self.get_parameter("set_lift_topic").value,
            10,
        )

        self.chassis_enable_pub = self.create_publisher(
            Bool,
            self.get_parameter("chassis_align_enable_topic").value,
            10,
        )

        self.lift_enable_pub = self.create_publisher(
            Bool,
            self.get_parameter("lift_enable_topic").value,
            10,
        )

        self.lift_freeze_pub = self.create_publisher(
            Bool,
            self.get_parameter("lift_freeze_topic").value,
            10,
        )

        self.extend_start_pub = self.create_publisher(
            Bool,
            self.get_parameter("pickup_extend_start_topic").value,
            10,
        )

        self.extend_retract_pub = self.create_publisher(
            Bool,
            self.get_parameter("pickup_extend_retract_topic").value,
            10,
        )

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(
            String,
            self.get_parameter("pickup_code_topic").value,
            self.pickup_code_cb,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter("wide_qr_state_topic").value,
            self.wide_state_cb,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter("wide_qr_raw_topic").value,
            self.wide_raw_cb,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter("pick_qr_state_topic").value,
            self.pick_state_cb,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter("pick_qr_raw_topic").value,
            self.pick_raw_cb,
            10,
        )

        self.create_subscription(
            PointStamped,
            self.get_parameter("pick_target_pose_topic").value,
            self.pose_cb,
            10,
        )

        self.create_timer(
            float(self.get_parameter("state_period_sec").value),
            self.loop,
        )

        self.get_logger().info(
            "parcel_pickup_mission_node started | "
            "mode=nav-arrived-local-pickup | "
            f"camera_mode={self.camera_mode()}"
        )

        self.set_state(MissionState.IDLE, "waiting pickup code")

    # =========================
    # Basic helpers
    # =========================
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        return self.now_sec() - self.state_enter_time

    def camera_mode(self):
        return str(self.get_parameter("camera_mode").value).strip()

    def use_wide_search(self):
        return self.camera_mode() == "usb_wide_oak_pick"

    def mirror_pick_target(self):
        return bool(self.get_parameter("mirror_pick_target_during_search").value)

    def log(self, text):
        msg = String()
        msg.data = str(text)
        self.debug_pub.publish(msg)
        self.get_logger().info(str(text))

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def set_state(self, new_state, reason=""):
        if self.state == new_state:
            return

        old = self.state
        self.state = new_state
        self.state_enter_time = self.now_sec()
        self.x_stable_count = 0
        self.y_stable_count = 0

        self.publish_state()
        self.log(f"STATE {old.value} -> {new_state.value} | {reason}")

    # =========================
    # Callbacks
    # =========================
    def pickup_code_cb(self, msg):
        self.pickup_code = str(msg.data).strip()

        if self.pickup_code:
            self.stop_all_motion()
            self.set_state(
                MissionState.PARSE_PICKUP_CODE,
                f"pickup_code={self.pickup_code}",
            )

    def wide_state_cb(self, msg):
        self.wide_state = str(msg.data).strip()

    def wide_raw_cb(self, msg):
        self.wide_raw = str(msg.data).strip()

    def pick_state_cb(self, msg):
        self.pick_state = str(msg.data).strip()

    def pick_raw_cb(self, msg):
        self.pick_raw = str(msg.data).strip()

    def pose_cb(self, msg):
        self.pose = msg
        self.pose_time = self.get_clock().now()

    # =========================
    # Publish helpers
    # =========================
    def publish_bool(self, pub, value):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def publish_wide_target(self, code):
        msg = String()
        msg.data = str(code)
        self.wide_target_pub.publish(msg)
        self.log(f"publish /wide_target_code = {code}")

    def publish_pick_target(self, code):
        msg = String()
        msg.data = str(code)
        self.pick_target_pub.publish(msg)
        self.log(f"publish /target_code = {code}")

    def publish_search_target(self, code):
        if self.use_wide_search():
            self.publish_wide_target(code)

            if self.mirror_pick_target():
                self.publish_pick_target(code)
                self.pick_raw = ""
                self.pick_state = ""
        else:
            self.publish_pick_target(code)
            self.pick_raw = ""
            self.pick_state = ""

    def publish_set_lift(self, height_mm):
        msg = Float32()
        msg.data = float(height_mm)
        self.set_lift_pub.publish(msg)
        self.log(f"publish /qr_lift_control/set_lift_mm = {height_mm:.1f}")

    def publish_cmd_vel(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def stop_chassis_motion(self):
        self.publish_cmd_vel(0.0, 0.0, 0.0)

    def stop_align(self):
        self.publish_bool(self.chassis_enable_pub, False)
        self.publish_bool(self.lift_enable_pub, False)

    def stop_all_motion(self):
        self.stop_align()
        self.stop_chassis_motion()

    # =========================
    # QR helpers
    # =========================
    def normalize_raw(self, raw):
        text = str(raw).strip()

        if text == "":
            return ""

        try:
            data = json.loads(text)
            if isinstance(data, dict):
                if "col" in data:
                    return f"COL:{int(data['col'])}"
                if "row" in data:
                    return f"ROW:{int(data['row'])}"
                if "pkg" in data:
                    return str(data["pkg"]).strip()
                if "package" in data:
                    return str(data["package"]).strip()
                if "id" in data:
                    return str(data["id"]).strip()
        except Exception:
            pass

        upper = text.upper()

        if upper.startswith("COL:"):
            return upper

        if upper.startswith("ROW:"):
            return upper

        if upper.startswith("PKG:"):
            return upper.split(":", 1)[1].strip()

        return text

    def search_state(self):
        if self.use_wide_search():
            return self.wide_state
        return self.pick_state

    def search_raw(self):
        if self.use_wide_search():
            return self.wide_raw
        return self.pick_raw

    def search_matches(self, expected):
        return (
            self.search_state() == "QR_TRACKING"
            and self.normalize_raw(self.search_raw()) == str(expected).strip()
        )

    def pick_matches(self, expected):
        return (
            self.pick_state == "QR_TRACKING"
            and self.normalize_raw(self.pick_raw) == str(expected).strip()
        )

    # =========================
    # Pickup code / row height
    # =========================
    def parse_pickup_code(self):
        text = str(self.pickup_code).strip()
        parts = text.split("-")

        if len(parts) != 3:
            return False, f"invalid pickup code format: {text}"

        col_s, row_s, pkg_s = [x.strip() for x in parts]

        if not (col_s.isdigit() and row_s.isdigit() and pkg_s.isdigit()):
            return False, f"invalid pickup code digits: {text}"

        self.col = int(col_s)
        self.row = int(row_s)
        self.package_code = pkg_s
        self.row_target_height_mm = self.get_row_height_mm(self.row)

        return True, (
            f"parsed pickup_code={self.pickup_code}, "
            f"col={self.col}, row={self.row}, package={self.package_code}, "
            f"row_height={self.row_target_height_mm:.1f}mm"
        )

    def get_row_height_mm(self, row):
        row_ids = list(self.get_parameter("row_ids").value)
        row_heights = list(self.get_parameter("row_heights_mm").value)
        row_str = str(row)

        for i, rid in enumerate(row_ids):
            if str(rid) == row_str and i < len(row_heights):
                return float(row_heights[i])

        base = float(self.get_parameter("base_height_mm").value)
        gap = float(self.get_parameter("level_gap_mm").value)

        return base + (float(row) - 1.0) * gap

    def parse_row_qr_height(self):
        raw = self.search_raw()

        try:
            data = json.loads(raw)
        except Exception:
            return None

        if not isinstance(data, dict):
            return None

        if "target_lift_mm" in data:
            return float(data["target_lift_mm"])

        if "base_height_mm" in data and "level_gap_mm" in data and "row" in data:
            return (
                float(data["base_height_mm"])
                + (float(data["row"]) - 1.0) * float(data["level_gap_mm"])
            )

        return None

    # =========================
    # TF anchor
    # =========================
    def quat_to_yaw(self, q):
        import math

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def record_current_anchor(self):
        map_frame = str(self.get_parameter("nav_frame_id").value)
        base_frame = str(self.get_parameter("base_frame_id").value)

        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
            )

            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            yaw = self.quat_to_yaw(tf.transform.rotation)

            self.col_anchor = (x, y, yaw)

            self.log(
                f"recorded current pose as COL:{self.col} anchor | "
                f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )
            return True

        except Exception as e:
            self.log(f"record anchor TF failed: {map_frame}->{base_frame}: {e}")

            if bool(self.get_parameter("require_tf_anchor").value):
                return False

            self.col_anchor = None
            self.log("continue without TF anchor because require_tf_anchor=false")
            return True

    # =========================
    # Pose / alignment
    # =========================
    def has_recent_pose(self):
        if self.pose is None or self.pose_time is None:
            return False

        dt = (self.get_clock().now() - self.pose_time).nanoseconds * 1e-9
        return dt <= float(self.get_parameter("pose_timeout_sec").value)

    def current_pose_mm(self):
        if not self.has_recent_pose():
            return None

        return {
            "x": float(self.pose.point.x) * 1000.0,
            "y": float(self.pose.point.y) * 1000.0,
            "z": float(self.pose.point.z) * 1000.0,
        }

    def x_aligned(self):
        p = self.current_pose_mm()

        if p is None:
            self.x_stable_count = 0
            return False

        target = float(self.get_parameter("x_target_mm").value)
        deadband = float(self.get_parameter("x_deadband_mm").value)
        err = p["x"] - target

        if abs(err) <= deadband:
            self.x_stable_count += 1
        else:
            self.x_stable_count = 0

        req = int(self.get_parameter("stable_required_frames").value)

        self.log(
            f"x_align x={p['x']:.1f} target={target:.1f} "
            f"err={err:.1f} stable={self.x_stable_count}/{req}"
        )

        return self.x_stable_count >= req

    def y_aligned(self):
        p = self.current_pose_mm()

        if p is None:
            self.y_stable_count = 0
            return False

        target = float(self.get_parameter("y_target_mm").value)
        deadband = float(self.get_parameter("y_deadband_mm").value)
        err = p["y"] - target

        if abs(err) <= deadband:
            self.y_stable_count += 1
        else:
            self.y_stable_count = 0

        req = int(self.get_parameter("stable_required_frames").value)

        self.log(
            f"y_align y={p['y']:.1f} target={target:.1f} "
            f"err={err:.1f} stable={self.y_stable_count}/{req}"
        )

        return self.y_stable_count >= req

    # =========================
    # Lift scan
    # =========================
    def update_lift_scan(self):
        if not bool(self.get_parameter("package_search_enable_lift_scan").value):
            return

        now = self.now_sec()
        interval = float(self.get_parameter("package_search_interval_sec").value)

        if now - self.last_search_lift_time < interval:
            return

        self.last_search_lift_time = now

        scan_range = float(self.get_parameter("package_search_range_mm").value)
        step = float(self.get_parameter("package_search_step_mm").value)
        lift_min = float(self.get_parameter("lift_min_mm").value)
        lift_max = float(self.get_parameter("lift_max_mm").value)

        low = max(lift_min, self.search_lift_center_mm - scan_range)
        high = min(lift_max, self.search_lift_center_mm + scan_range)

        if high < low:
            high = low

        if self.search_lift_cmd_mm < low or self.search_lift_cmd_mm > high:
            self.search_lift_cmd_mm = max(low, min(high, self.search_lift_center_mm))

        self.search_lift_cmd_mm += self.search_lift_dir * step
        self.search_lift_cmd_mm = max(low, min(high, self.search_lift_cmd_mm))

        if self.search_lift_cmd_mm >= high:
            self.search_lift_cmd_mm = high
            self.search_lift_dir = -1

        if self.search_lift_cmd_mm <= low:
            self.search_lift_cmd_mm = low
            self.search_lift_dir = 1

        self.publish_set_lift(self.search_lift_cmd_mm)

        self.log(
            f"package search lift scan: {self.search_lift_cmd_mm:.1f}mm "
            f"range=[{low:.1f},{high:.1f}] center={self.search_lift_center_mm:.1f}"
        )

    # =========================
    # State loop
    # =========================
    def loop(self):
        self.publish_state()

        if self.state == MissionState.IDLE:
            return

        if self.state == MissionState.PARSE_PICKUP_CODE:
            ok, info = self.parse_pickup_code()

            if not ok:
                self.set_state(MissionState.ERROR, info)
                return

            self.log(info)
            self.stop_all_motion()
            self.publish_bool(self.lift_freeze_pub, False)
            self.set_state(MissionState.RECORD_CURRENT_ANCHOR, "pickup code parsed")
            return

        if self.state == MissionState.RECORD_CURRENT_ANCHOR:
            self.stop_chassis_motion()

            if not self.record_current_anchor():
                self.set_state(MissionState.ERROR, "failed to record current anchor")
                return

            self.set_state(MissionState.SET_COL_TARGET, "current pose recorded as anchor")
            return

        # -------------------------
        # Verify COL at current area
        # -------------------------
        if self.state == MissionState.SET_COL_TARGET:
            target = f"{self.get_parameter('shelf_col_prefix').value}{self.col}"
            self.publish_search_target(target)

            self.wide_raw = ""
            self.wide_state = ""
            self.pick_raw = ""
            self.pick_state = ""

            self.publish_bool(self.chassis_enable_pub, False)
            self.publish_bool(self.lift_enable_pub, False)
            self.publish_bool(self.lift_freeze_pub, False)

            self.set_state(MissionState.VERIFY_COL_AT_ANCHOR, f"verify {target}")
            return

        if self.state == MissionState.VERIFY_COL_AT_ANCHOR:
            self.stop_chassis_motion()

            target = f"{self.get_parameter('shelf_col_prefix').value}{self.col}"

            if not bool(self.get_parameter("require_col_verify").value):
                self.set_state(MissionState.SET_ROW_TARGET, "skip col verify")
                return

            if self.search_matches(target):
                self.set_state(MissionState.SET_ROW_TARGET, "col verified")
                return

            if self.elapsed() >= float(self.get_parameter("col_verify_timeout_sec").value):
                self.set_state(
                    MissionState.ERROR,
                    f"col verify timeout: expected={target}, raw={self.search_raw()}",
                )
                return

            return

        # -------------------------
        # Verify ROW at current area
        # -------------------------
        if self.state == MissionState.SET_ROW_TARGET:
            target = f"{self.get_parameter('shelf_row_prefix').value}{self.row}"
            self.publish_search_target(target)

            self.wide_raw = ""
            self.wide_state = ""
            self.pick_raw = ""
            self.pick_state = ""

            self.set_state(MissionState.VERIFY_ROW_AT_ANCHOR, f"verify {target}")
            return

        if self.state == MissionState.VERIFY_ROW_AT_ANCHOR:
            self.stop_chassis_motion()

            target = f"{self.get_parameter('shelf_row_prefix').value}{self.row}"

            if self.search_matches(target):
                h = self.parse_row_qr_height()

                if h is not None:
                    self.row_target_height_mm = max(0.0, h)
                    self.log(f"row QR height override: {self.row_target_height_mm:.1f}mm")

                self.set_state(MissionState.COARSE_LIFT_TO_ROW, "row verified")
                return

            # 容错：如果原地已经看到包裹码，使用配置表高度继续
            if self.normalize_raw(self.search_raw()) == str(self.package_code):
                self.log(
                    f"package {self.package_code} seen while verifying row; "
                    f"use configured row_height={self.row_target_height_mm:.1f}mm"
                )
                self.set_state(MissionState.COARSE_LIFT_TO_ROW, "package seen before row")
                return

            if self.elapsed() >= float(self.get_parameter("row_verify_timeout_sec").value):
                self.set_state(
                    MissionState.ERROR,
                    f"row verify timeout: expected={target}, raw={self.search_raw()}",
                )
                return

            return

        # -------------------------
        # Coarse lift
        # -------------------------
        if self.state == MissionState.COARSE_LIFT_TO_ROW:
            self.stop_chassis_motion()
            self.publish_bool(self.lift_enable_pub, False)
            self.publish_bool(self.lift_freeze_pub, False)

            target_h = max(0.0, float(self.row_target_height_mm))
            self.row_target_height_mm = target_h

            self.publish_set_lift(target_h)

            self.search_lift_center_mm = float(target_h)
            self.search_lift_cmd_mm = float(target_h)
            self.search_lift_dir = 1
            self.last_search_lift_time = 0.0

            self.set_state(
                MissionState.WAIT_ROW_HEIGHT,
                f"coarse lift to {self.row_target_height_mm:.1f}mm",
            )
            return

        if self.state == MissionState.WAIT_ROW_HEIGHT:
            self.stop_chassis_motion()

            if self.elapsed() >= float(self.get_parameter("row_height_wait_sec").value):
                self.set_state(MissionState.SET_PACKAGE_SEARCH_TARGET, "row height wait done")
                return

            return

        # -------------------------
        # Search package at current area
        # -------------------------
        if self.state == MissionState.SET_PACKAGE_SEARCH_TARGET:
            self.publish_search_target(self.package_code)

            self.wide_raw = ""
            self.wide_state = ""
            self.pick_raw = ""
            self.pick_state = ""

            self.publish_bool(self.lift_freeze_pub, False)
            self.publish_bool(self.lift_enable_pub, False)
            self.publish_bool(self.chassis_enable_pub, False)

            self.search_lift_center_mm = float(self.row_target_height_mm)
            self.search_lift_cmd_mm = float(self.row_target_height_mm)
            self.search_lift_dir = 1
            self.last_search_lift_time = 0.0

            self.set_state(
                MissionState.SEARCH_PACKAGE_AT_ANCHOR,
                f"search package {self.package_code} at current area",
            )
            return

        if self.state == MissionState.SEARCH_PACKAGE_AT_ANCHOR:
            self.stop_chassis_motion()
            self.update_lift_scan()

            # OAK 优先接管
            if self.pick_matches(self.package_code) and self.has_recent_pose():
                self.log(f"OAK sees package {self.package_code}; enter chassis fine align")
                self.set_state(MissionState.CHASSIS_FINE_ALIGN, "package found by OAK")
                return

            # USB 广角找到包裹
            if self.search_matches(self.package_code):
                self.set_state(MissionState.SET_PICK_PACKAGE_TARGET, "package found by wide camera")
                return

            if self.elapsed() >= float(self.get_parameter("package_search_timeout_sec").value):
                self.set_state(
                    MissionState.ERROR,
                    f"package search timeout: expected={self.package_code}, raw={self.search_raw()}",
                )
                return

            return

        # -------------------------
        # OAK final view
        # -------------------------
        if self.state == MissionState.SET_PICK_PACKAGE_TARGET:
            self.publish_pick_target(self.package_code)
            self.pick_raw = ""
            self.pick_state = ""

            self.publish_bool(self.lift_freeze_pub, False)
            self.publish_bool(self.lift_enable_pub, False)
            self.publish_bool(self.chassis_enable_pub, False)

            self.set_state(MissionState.WAIT_OAK_VIEW, "wait OAK sees package")
            return

        if self.state == MissionState.WAIT_OAK_VIEW:
            self.stop_chassis_motion()

            if self.pick_matches(self.package_code) and self.has_recent_pose():
                self.set_state(MissionState.CHASSIS_FINE_ALIGN, "OAK package confirmed")
                return

            if self.elapsed() > float(self.get_parameter("oak_view_timeout_sec").value):
                self.set_state(
                    MissionState.ERROR,
                    f"OAK view timeout: expected={self.package_code}, raw={self.pick_raw}",
                )
                return

            return

        # -------------------------
        # Fine align
        # -------------------------
        if self.state == MissionState.CHASSIS_FINE_ALIGN:
            self.stop_chassis_motion()
            self.publish_bool(self.chassis_enable_pub, True)

            if self.x_aligned():
                self.publish_bool(self.chassis_enable_pub, False)
                self.set_state(MissionState.LIFT_FINE_ALIGN, "x aligned")
                return

            if self.elapsed() > float(self.get_parameter("chassis_align_timeout_sec").value):
                self.publish_bool(self.chassis_enable_pub, False)
                self.set_state(MissionState.ERROR, "chassis align timeout")
                return

            return

        if self.state == MissionState.LIFT_FINE_ALIGN:
            self.publish_bool(self.lift_freeze_pub, False)
            self.publish_bool(self.lift_enable_pub, True)

            if self.y_aligned():
                self.publish_bool(self.lift_enable_pub, False)
                self.set_state(MissionState.FREEZE_LIFT, "y aligned")
                return

            if self.elapsed() > float(self.get_parameter("lift_align_timeout_sec").value):
                self.publish_bool(self.lift_enable_pub, False)
                self.set_state(MissionState.ERROR, "lift align timeout")
                return

            return

        if self.state == MissionState.FREEZE_LIFT:
            self.stop_chassis_motion()
            self.publish_bool(self.lift_enable_pub, False)
            self.publish_bool(self.lift_freeze_pub, True)
            self.set_state(MissionState.EXTEND_PLATFORM_AND_HAND, "freeze lift")
            return

        # -------------------------
        # Extend / retract
        # -------------------------
        if self.state == MissionState.EXTEND_PLATFORM_AND_HAND:
            self.publish_bool(self.extend_start_pub, True)
            self.set_state(MissionState.WAIT_EXTENDED, "extend start")
            return

        if self.state == MissionState.WAIT_EXTENDED:
            if self.elapsed() >= float(self.get_parameter("extend_wait_sec").value):
                self.set_state(MissionState.RETRACT_HAND_AND_PLATFORM, "extend wait done")
                return

            return

        if self.state == MissionState.RETRACT_HAND_AND_PLATFORM:
            self.publish_bool(self.extend_retract_pub, True)
            self.set_state(MissionState.WAIT_RETRACTED, "retract start")
            return

        if self.state == MissionState.WAIT_RETRACTED:
            if self.elapsed() >= float(self.get_parameter("retract_wait_sec").value):
                self.set_state(MissionState.DONE, "retract wait done")
                return

            return

        if self.state == MissionState.DONE:
            self.stop_all_motion()
            self.publish_bool(self.lift_freeze_pub, False)
            self.log(f"DONE pickup_code={self.pickup_code}")
            self.set_state(MissionState.IDLE, "mission done")
            return

        if self.state == MissionState.ERROR:
            self.stop_all_motion()
            self.publish_bool(self.lift_enable_pub, False)
            self.publish_bool(self.lift_freeze_pub, True)
            return


def main(args=None):
    rclpy.init(args=args)
    node = ParcelPickupMissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all_motion()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
