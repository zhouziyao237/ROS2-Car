#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BTOSDOverlay(Node):
    def __init__(self):
        super().__init__("bt_osd_overlay")

        self.declare_parameter("input_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/task/bt_osd/image_raw")
        self.declare_parameter("current_state_topic", "/task/current_state")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("current_goal_topic", "/task/current_goal")
        self.declare_parameter("trace_topic", "/task/bt_trace")

        self.declare_parameter("draw_goal", True)
        self.declare_parameter("draw_trace", True)
        self.declare_parameter("draw_status", True)
        self.declare_parameter("font_scale", 0.55)
        self.declare_parameter("thickness", 1)

        self.input_image_topic = self.get_parameter("input_image_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value

        self.bridge = CvBridge()

        self.current_state = "IDLE"
        self.status = ""
        self.current_goal = ""
        self.last_trace = ""
        self.last_update_time = time.time()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        text_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.on_image,
            image_qos,
        )

        self.state_sub = self.create_subscription(
            String,
            self.get_parameter("current_state_topic").value,
            self.on_state,
            text_qos,
        )

        self.status_sub = self.create_subscription(
            String,
            self.get_parameter("status_topic").value,
            self.on_status,
            text_qos,
        )

        self.goal_sub = self.create_subscription(
            String,
            self.get_parameter("current_goal_topic").value,
            self.on_goal,
            text_qos,
        )

        self.trace_sub = self.create_subscription(
            String,
            self.get_parameter("trace_topic").value,
            self.on_trace,
            text_qos,
        )

        self.image_pub = self.create_publisher(
            Image,
            self.output_image_topic,
            image_qos,
        )

        self.get_logger().info(
            f"BT OSD overlay started: {self.input_image_topic} -> {self.output_image_topic}"
        )

    def on_state(self, msg: String):
        self.current_state = msg.data
        self.last_update_time = time.time()

    def on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "")
            status = data.get("status", "")
            message = data.get("message", "")
            self.status = f"{status} | {state} | {message}"
        except Exception:
            self.status = msg.data
        self.last_update_time = time.time()

    def on_goal(self, msg: String):
        self.current_goal = msg.data
        self.last_update_time = time.time()

    def on_trace(self, msg: String):
        try:
            data = json.loads(msg.data)
            node = data.get("node", "")
            result = data.get("result", "")
            self.last_trace = f"{node} -> {result}"
        except Exception:
            self.last_trace = msg.data
        self.last_update_time = time.time()

    def draw_panel(self, frame, lines):
        h, w = frame.shape[:2]

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = float(self.get_parameter("font_scale").value)
        thickness = int(self.get_parameter("thickness").value)

        line_height = int(24 * font_scale / 0.55)
        top = 10
        left = 10
        panel_height = 20 + line_height * len(lines)
        panel_width = min(w - 20, 900)

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (left, top),
            (left + panel_width, top + panel_height),
            (0, 0, 0),
            -1,
        )
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        y = top + 25
        for text, color in lines:
            cv2.putText(
                frame,
                text[:120],
                (left + 12, y),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )
            y += line_height

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        age = time.time() - self.last_update_time

        lines = []

        lines.append((f"BT STATE: {self.current_state}", (0, 255, 255)))

        if self.get_parameter("draw_status").value:
            lines.append((f"STATUS: {self.status}", (255, 255, 255)))

        if self.get_parameter("draw_trace").value:
            lines.append((f"TRACE: {self.last_trace}", (180, 255, 180)))

        if self.get_parameter("draw_goal").value:
            goal = self.current_goal
            if len(goal) > 120:
                goal = goal[:117] + "..."
            lines.append((f"GOAL: {goal}", (200, 200, 255)))

        lines.append((f"BT UPDATE AGE: {age:.1f}s", (180, 180, 180)))

        self.draw_panel(frame, lines)

        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self.image_pub.publish(out)


def main():
    rclpy.init()
    node = BTOSDOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
