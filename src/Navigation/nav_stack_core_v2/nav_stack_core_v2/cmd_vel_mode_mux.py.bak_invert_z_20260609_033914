#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def finite_twist(msg: Twist) -> bool:
    vals = [
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    ]
    return all(math.isfinite(float(v)) for v in vals)


class CmdVelModeMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mode_mux")

        self.declare_parameter("nav_cmd_topics", [
            "/cmd_vel",
            "/cmd_vel_smoothed",
            "/cmd_vel_nav",
            "/cmd_vel_auto",
        ])
        self.declare_parameter("manual_cmd_topics", [
            "/cmd_vel_manual",
            "/cmd_vel_keyboard",
            "/cmd_vel_teleop",
            "/cmd_vel_manual_raw",
        ])

        self.declare_parameter("output_cmd_topic", "/cmd_vel_selected")
        self.declare_parameter("control_mode_topic", "/control_mode")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("nav_timeout_sec", 0.8)
        self.declare_parameter("manual_timeout_sec", 0.8)
        self.declare_parameter("default_mode", "auto")

        # 保留你之前底盘方向修正。
        self.declare_parameter("swap_xy", True)
        self.declare_parameter("invert_x", True)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("invert_z", False)

        self.declare_parameter("deadband_linear", 0.0)
        self.declare_parameter("deadband_angular", 0.0)

        self.declare_parameter("debug_log", True)
        self.declare_parameter("debug_period_sec", 0.5)

        self.mode = str(self.get_parameter("default_mode").value).strip().lower()

        self.nav_topics: List[str] = list(self.get_parameter("nav_cmd_topics").value)
        self.manual_topics: List[str] = list(self.get_parameter("manual_cmd_topics").value)
        self.output_topic = str(self.get_parameter("output_cmd_topic").value)
        self.control_mode_topic = str(self.get_parameter("control_mode_topic").value)

        self.last_nav: Dict[str, Twist] = {}
        self.last_nav_time: Dict[str, float] = {}

        self.last_manual: Dict[str, Twist] = {}
        self.last_manual_time: Dict[str, float] = {}

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.subs = []

        for topic in self.nav_topics:
            if topic == self.output_topic:
                continue
            self.subs.append(
                self.create_subscription(
                    Twist,
                    topic,
                    lambda msg, t=topic: self.on_nav_cmd(t, msg),
                    10,
                )
            )

        for topic in self.manual_topics:
            if topic == self.output_topic:
                continue
            self.subs.append(
                self.create_subscription(
                    Twist,
                    topic,
                    lambda msg, t=topic: self.on_manual_cmd(t, msg),
                    10,
                )
            )

        self.mode_sub = self.create_subscription(
            String,
            self.control_mode_topic,
            self.on_mode,
            10,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.on_timer)

        self.last_debug_time = 0.0

        self.get_logger().info(
            f"cmd_vel_mode_mux started: nav_topics={self.nav_topics}, "
            f"manual_topics={self.manual_topics}, output={self.output_topic}, "
            f"mode={self.mode}, swap_xy={self.get_parameter('swap_xy').value}, "
            f"invert_x={self.get_parameter('invert_x').value}, "
            f"invert_y={self.get_parameter('invert_y').value}, "
            f"invert_z={self.get_parameter('invert_z').value}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_mode(self, msg: String):
        m = msg.data.strip().lower()

        if m in ["auto", "nav", "navigation"]:
            self.mode = "auto"
        elif m in ["manual", "teleop", "keyboard"]:
            self.mode = "manual"
        elif m in ["stop", "idle", "zero"]:
            self.mode = "stop"
        else:
            self.get_logger().warn(f"unknown control mode: {msg.data}")

    def on_nav_cmd(self, topic: str, msg: Twist):
        if not finite_twist(msg):
            self.get_logger().warn(f"ignore non-finite nav cmd from {topic}")
            return

        self.last_nav[topic] = msg
        self.last_nav_time[topic] = self.now_sec()

    def on_manual_cmd(self, topic: str, msg: Twist):
        if not finite_twist(msg):
            self.get_logger().warn(f"ignore non-finite manual cmd from {topic}")
            return

        self.last_manual[topic] = msg
        self.last_manual_time[topic] = self.now_sec()

    def freshest(self, store: Dict[str, Twist], times: Dict[str, float], timeout: float):
        now = self.now_sec()
        best_topic: Optional[str] = None
        best_time = -1.0

        for topic, stamp in times.items():
            if topic not in store:
                continue
            if now - stamp <= timeout and stamp > best_time:
                best_topic = topic
                best_time = stamp

        if best_topic is None:
            return None, "", 999.0

        return store[best_topic], best_topic, now - best_time

    def apply_transform(self, msg: Twist) -> Twist:
        out = Twist()

        x = float(msg.linear.x)
        y = float(msg.linear.y)
        wz = float(msg.angular.z)

        if bool(self.get_parameter("swap_xy").value):
            x, y = y, x

        if bool(self.get_parameter("invert_x").value):
            x = -x
        if bool(self.get_parameter("invert_y").value):
            y = -y
        if bool(self.get_parameter("invert_z").value):
            wz = -wz

        dead_l = float(self.get_parameter("deadband_linear").value)
        dead_a = float(self.get_parameter("deadband_angular").value)

        if abs(x) < dead_l:
            x = 0.0
        if abs(y) < dead_l:
            y = 0.0
        if abs(wz) < dead_a:
            wz = 0.0

        out.linear.x = x
        out.linear.y = y
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = wz

        return out

    def zero(self) -> Twist:
        return Twist()

    def on_timer(self):
        nav_timeout = float(self.get_parameter("nav_timeout_sec").value)
        manual_timeout = float(self.get_parameter("manual_timeout_sec").value)

        raw = None
        selected = self.zero()
        source = "zero"
        age = 999.0

        if self.mode == "manual":
            raw, topic, age = self.freshest(
                self.last_manual,
                self.last_manual_time,
                manual_timeout,
            )
            if raw is not None:
                selected = self.apply_transform(raw)
                source = f"manual:{topic}"
            else:
                source = "manual_timeout"

        elif self.mode == "auto":
            raw, topic, age = self.freshest(
                self.last_nav,
                self.last_nav_time,
                nav_timeout,
            )
            if raw is not None:
                selected = self.apply_transform(raw)
                source = f"auto:{topic}"
            else:
                source = "auto_timeout"

        else:
            selected = self.zero()
            source = "stop_mode"

        self.pub.publish(selected)

        debug = bool(self.get_parameter("debug_log").value)
        period = float(self.get_parameter("debug_period_sec").value)
        now = self.now_sec()

        if debug and now - self.last_debug_time >= period:
            self.last_debug_time = now

            if raw is None:
                self.get_logger().info(
                    f"[cmd_mux] mode={self.mode}, source={source}, "
                    f"out vx={selected.linear.x:.3f}, "
                    f"vy={selected.linear.y:.3f}, "
                    f"wz={selected.angular.z:.3f}"
                )
            else:
                self.get_logger().info(
                    f"[cmd_mux] mode={self.mode}, source={source}, age={age:.3f}, "
                    f"raw vx={raw.linear.x:.3f}, vy={raw.linear.y:.3f}, wz={raw.angular.z:.3f} "
                    f"-> out vx={selected.linear.x:.3f}, "
                    f"vy={selected.linear.y:.3f}, "
                    f"wz={selected.angular.z:.3f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeMux()

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
