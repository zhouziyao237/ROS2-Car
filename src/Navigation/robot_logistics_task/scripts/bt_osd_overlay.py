#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from typing import Any, Dict, Optional, Tuple

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped


def safe_json_loads(text: str) -> Dict[str, Any]:
    try:
        v = json.loads(text)
        if isinstance(v, dict):
            return v
    except Exception:
        pass
    return {"raw": text}


def now_sec(node: Node) -> float:
    return node.get_clock().now().nanoseconds / 1e9


def age_text(node: Node, stamp: float) -> str:
    if stamp <= 0.0:
        return "never"
    a = now_sec(node) - stamp
    if a < 0.0:
        a = 0.0
    return f"{a:.1f}s"


def twist_text(msg: Optional[Twist]) -> str:
    if msg is None:
        return "vx=-- vy=-- wz=--"
    return f"vx={msg.linear.x:+.3f} vy={msg.linear.y:+.3f} wz={msg.angular.z:+.3f}"


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class BtOsdOverlay(Node):
    def __init__(self):
        super().__init__("bt_osd_overlay")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/task/bt_osd/image_raw")

        self.declare_parameter("task_status_topic", "/task/status")
        self.declare_parameter("current_state_topic", "/task/current_state")
        self.declare_parameter("current_goal_topic", "/task/current_goal")
        self.declare_parameter("bt_trace_topic", "/task/bt_trace")

        self.declare_parameter("semantic_observation_topic", "/vision/semantic_observation")
        self.declare_parameter("current_shelf_topic", "/semantic/current_shelf")

        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("selected_cmd_topic", "/cmd_vel_selected")
        self.declare_parameter("control_mode_topic", "/control_mode")

        self.declare_parameter("nav_cmd_topics", [
            "/cmd_vel",
            "/cmd_vel_smoothed",
            "/cmd_vel_nav",
            "/cmd_vel_auto",
        ])

        self.declare_parameter("font_scale", 0.46)
        self.declare_parameter("line_height", 18)
        self.declare_parameter("panel_alpha", 0.58)
        self.declare_parameter("max_lines", 22)

        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        normal_qos = QoSProfile(depth=10)

        self.pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_image_topic").value),
            sensor_qos,
        )

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )

        self.latest_task_status = {}
        self.latest_task_status_stamp = 0.0

        self.latest_state = "IDLE"
        self.latest_state_stamp = 0.0

        self.latest_goal = {}
        self.latest_goal_stamp = 0.0

        self.latest_trace = ""
        self.latest_trace_stamp = 0.0

        self.latest_obs = {}
        self.latest_obs_stamp = 0.0

        self.latest_current_shelf = ""
        self.latest_current_shelf_stamp = 0.0

        self.latest_goal_pose: Optional[PoseStamped] = None
        self.latest_goal_pose_stamp = 0.0

        self.latest_selected_cmd: Optional[Twist] = None
        self.latest_selected_cmd_stamp = 0.0

        self.latest_nav_cmds: Dict[str, Twist] = {}
        self.latest_nav_cmd_stamps: Dict[str, float] = {}

        self.latest_mode = "auto"
        self.latest_mode_stamp = 0.0

        self.subs = [
            self.create_subscription(String, str(self.get_parameter("task_status_topic").value), self.on_task_status, normal_qos),
            self.create_subscription(String, str(self.get_parameter("current_state_topic").value), self.on_state, normal_qos),
            self.create_subscription(String, str(self.get_parameter("current_goal_topic").value), self.on_goal_json, normal_qos),
            self.create_subscription(String, str(self.get_parameter("bt_trace_topic").value), self.on_trace, normal_qos),
            self.create_subscription(String, str(self.get_parameter("semantic_observation_topic").value), self.on_obs, normal_qos),
            self.create_subscription(String, str(self.get_parameter("current_shelf_topic").value), self.on_current_shelf, normal_qos),
            self.create_subscription(String, str(self.get_parameter("control_mode_topic").value), self.on_mode, normal_qos),
            self.create_subscription(PoseStamped, str(self.get_parameter("goal_pose_topic").value), self.on_goal_pose, normal_qos),
            self.create_subscription(Twist, str(self.get_parameter("selected_cmd_topic").value), self.on_selected_cmd, normal_qos),
        ]

        for topic in list(self.get_parameter("nav_cmd_topics").value):
            self.subs.append(
                self.create_subscription(
                    Twist,
                    topic,
                    lambda msg, t=topic: self.on_nav_cmd(t, msg),
                    normal_qos,
                )
            )

        self.get_logger().info(
            f"bt_osd_overlay started: {self.get_parameter('image_topic').value} "
            f"-> {self.get_parameter('output_image_topic').value}"
        )

    def on_task_status(self, msg: String):
        self.latest_task_status = safe_json_loads(msg.data)
        self.latest_task_status_stamp = now_sec(self)

    def on_state(self, msg: String):
        self.latest_state = msg.data
        self.latest_state_stamp = now_sec(self)

    def on_goal_json(self, msg: String):
        self.latest_goal = safe_json_loads(msg.data)
        self.latest_goal_stamp = now_sec(self)

    def on_trace(self, msg: String):
        self.latest_trace = msg.data
        self.latest_trace_stamp = now_sec(self)

    def on_obs(self, msg: String):
        self.latest_obs = safe_json_loads(msg.data)
        self.latest_obs_stamp = now_sec(self)

    def on_current_shelf(self, msg: String):
        self.latest_current_shelf = msg.data
        self.latest_current_shelf_stamp = now_sec(self)

    def on_goal_pose(self, msg: PoseStamped):
        self.latest_goal_pose = msg
        self.latest_goal_pose_stamp = now_sec(self)

    def on_selected_cmd(self, msg: Twist):
        self.latest_selected_cmd = msg
        self.latest_selected_cmd_stamp = now_sec(self)

    def on_nav_cmd(self, topic: str, msg: Twist):
        self.latest_nav_cmds[topic] = msg
        self.latest_nav_cmd_stamps[topic] = now_sec(self)

    def on_mode(self, msg: String):
        self.latest_mode = msg.data
        self.latest_mode_stamp = now_sec(self)

    def newest_nav_cmd(self) -> Tuple[str, Optional[Twist], float]:
        best_topic = ""
        best_stamp = -1.0

        for topic, stamp in self.latest_nav_cmd_stamps.items():
            if stamp > best_stamp and topic in self.latest_nav_cmds:
                best_topic = topic
                best_stamp = stamp

        if not best_topic:
            return "", None, 999.0

        return best_topic, self.latest_nav_cmds[best_topic], now_sec(self) - best_stamp

    def short(self, text: Any, n: int = 66) -> str:
        s = str(text).replace("\n", " ")
        return s if len(s) <= n else s[:n - 3] + "..."

    def build_lines(self):
        lines = []
        colors = []

        lines.append("=== TASK / QR / SPEED OSD ===")
        colors.append((255, 255, 255))

        st = self.latest_task_status
        bt_state = st.get("state", self.latest_state)
        bt_status = st.get("status", "")
        bt_msg = st.get("message", "")

        lines.append(f"BT: {bt_state} | {bt_status} | age={age_text(self, self.latest_task_status_stamp)}")
        colors.append((0, 255, 255))

        if bt_msg:
            lines.append(f"MSG: {self.short(bt_msg)}")
            colors.append((0, 255, 255))

        goal = self.latest_goal
        shelf = goal.get("shelf_id", goal.get("target_shelf_id", ""))
        cell = goal.get("cell_id", goal.get("target_cell_id", ""))
        layer = goal.get("layer_id", goal.get("layer", goal.get("target_layer_id", "")))
        row = goal.get("row", "")
        col = goal.get("col", "")
        if shelf or cell:
            lines.append(f"TASK: shelf={shelf} cell={cell} layer={layer} row={row} col={col}")
            colors.append((0, 255, 255))

        if self.latest_trace:
            lines.append(f"TRACE: {self.short(self.latest_trace)}")
            colors.append((180, 220, 255))

        obs = self.latest_obs
        obs_shelf = obs.get("shelf_id", obs.get("shelf", ""))
        obs_layer = obs.get("layer_id", obs.get("layer", ""))
        obs_type = obs.get("type", obs.get("obs_type", ""))
        raw = obs.get("raw", obs.get("qr", obs.get("text", "")))
        source = obs.get("source", "")

        if obs_shelf or raw or obs_type:
            lines.append(f"QR: shelf={obs_shelf} layer={obs_layer} type={obs_type} raw={raw}")
            colors.append((0, 255, 0))
            lines.append(f"QR_SRC: {self.short(source)} age={age_text(self, self.latest_obs_stamp)}")
            colors.append((0, 255, 0))

        camera_xyz = obs.get("camera_xyz", None)
        if isinstance(camera_xyz, list) and len(camera_xyz) >= 3:
            lines.append(f"QR_XYZ: x={float(camera_xyz[0]):+.2f} y={float(camera_xyz[1]):+.2f} z={float(camera_xyz[2]):+.2f}")
            colors.append((0, 255, 0))

        normal = obs.get("normal", None)
        if isinstance(normal, list) and len(normal) >= 3:
            lines.append(f"QR_N: nx={float(normal[0]):+.2f} ny={float(normal[1]):+.2f} nz={float(normal[2]):+.2f}")
            colors.append((0, 255, 0))

        if self.latest_current_shelf:
            lines.append(f"CUR_SHELF: {self.short(self.latest_current_shelf)} age={age_text(self, self.latest_current_shelf_stamp)}")
            colors.append((80, 255, 80))

        if self.latest_goal_pose is not None:
            p = self.latest_goal_pose.pose.position
            yaw = yaw_from_pose(self.latest_goal_pose)
            lines.append(f"GOAL: x={p.x:+.2f} y={p.y:+.2f} yaw={yaw:+.2f} age={age_text(self, self.latest_goal_pose_stamp)}")
            colors.append((255, 180, 80))

        nav_topic, nav_cmd, nav_age = self.newest_nav_cmd()
        if nav_cmd is not None:
            lines.append(f"NAV_CMD[{nav_topic}]: {twist_text(nav_cmd)} age={nav_age:.1f}s")
            colors.append((80, 180, 255))
        else:
            lines.append("NAV_CMD: none")
            colors.append((80, 80, 255))

        lines.append(f"SEL_CMD: {twist_text(self.latest_selected_cmd)} age={age_text(self, self.latest_selected_cmd_stamp)}")
        colors.append((80, 180, 255))

        lines.append(f"MODE: {self.latest_mode} age={age_text(self, self.latest_mode_stamp)}")
        colors.append((80, 180, 255))

        max_lines = int(self.get_parameter("max_lines").value)
        return lines[:max_lines], colors[:max_lines]

    def draw_panel(self, frame, lines, colors):
        h, w = frame.shape[:2]
        font_scale = float(self.get_parameter("font_scale").value)
        line_h = int(self.get_parameter("line_height").value)
        alpha = float(self.get_parameter("panel_alpha").value)

        panel_w = min(w - 12, 820)
        panel_h = min(h - 12, 16 + line_h * len(lines) + 8)

        overlay = frame.copy()
        cv2.rectangle(overlay, (4, 4), (4 + panel_w, 4 + panel_h), (0, 0, 0), -1)
        frame[:] = cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)

        x = 10
        y0 = 24

        for i, line in enumerate(lines):
            y = y0 + i * line_h
            color = colors[i] if i < len(colors) else (255, 255, 255)
            cv2.putText(
                frame,
                line,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                color,
                1,
                cv2.LINE_AA,
            )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {repr(e)}")
            return

        lines, colors = self.build_lines()
        self.draw_panel(frame, lines, colors)

        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out.header = msg.header
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"image publish failed: {repr(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = BtOsdOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
