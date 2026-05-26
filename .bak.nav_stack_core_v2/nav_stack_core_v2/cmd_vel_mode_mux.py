#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelModeMux(Node):
    """
    Bottom-level velocity selector.

    Modes:
      manual:
        /cmd_vel_manual -> /cmd_vel_selected

      nav:
        /cmd_vel -> /cmd_vel_selected
        Used for RViz/Nav2 manual goal testing.

      auto:
        /cmd_vel -> /cmd_vel_selected
        Used for semantic_task_manager/frontier automatic goals.

      stop:
        zero Twist -> /cmd_vel_selected
    """

    def __init__(self):
        super().__init__('cmd_vel_mode_mux')

        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('nav_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('default_mode', 'stop')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('publish_rate', 30.0)

        self.mode_topic = self.get_parameter('mode_topic').value
        self.manual_topic = self.get_parameter('manual_topic').value
        self.nav_topic = self.get_parameter('nav_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mode = str(self.get_parameter('default_mode').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.valid_modes = {'manual', 'nav', 'auto', 'stop'}

        if self.mode not in self.valid_modes:
            self.get_logger().warn(
                f'invalid default_mode="{self.mode}", fallback to stop'
            )
            self.mode = 'stop'

        self.last_manual = Twist()
        self.last_nav = Twist()
        self.last_manual_time = None
        self.last_nav_time = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.sub_mode = self.create_subscription(
            String,
            self.mode_topic,
            self.cb_mode,
            10
        )

        self.sub_manual = self.create_subscription(
            Twist,
            self.manual_topic,
            self.cb_manual,
            10
        )

        self.sub_nav = self.create_subscription(
            Twist,
            self.nav_topic,
            self.cb_nav,
            10
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.tick
        )

        self.get_logger().info(
            f'cmd_vel_mode_mux started: '
            f'mode_topic={self.mode_topic}, '
            f'manual_topic={self.manual_topic}, '
            f'nav_topic={self.nav_topic}, '
            f'output_topic={self.output_topic}, '
            f'default_mode={self.mode}'
        )

    def cb_mode(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode not in self.valid_modes:
            self.get_logger().warn(
                f'ignore invalid control mode="{msg.data}". '
                f'valid={sorted(self.valid_modes)}'
            )
            return

        if new_mode != self.mode:
            self.get_logger().info(f'control mode: {self.mode} -> {new_mode}')

        self.mode = new_mode

    def cb_manual(self, msg: Twist):
        self.last_manual = msg
        self.last_manual_time = self.get_clock().now()

    def cb_nav(self, msg: Twist):
        self.last_nav = msg
        self.last_nav_time = self.get_clock().now()

    def is_fresh(self, stamp):
        if stamp is None:
            return False
        dt = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return dt <= self.timeout_sec

    @staticmethod
    def zero_twist():
        return Twist()

    def tick(self):
        out = self.zero_twist()

        if self.mode == 'stop':
            out = self.zero_twist()

        elif self.mode == 'manual':
            if self.is_fresh(self.last_manual_time):
                out = self.last_manual
            else:
                out = self.zero_twist()

        elif self.mode in ('nav', 'auto'):
            if self.is_fresh(self.last_nav_time):
                out = self.last_nav
            else:
                out = self.zero_twist()

        else:
            out = self.zero_twist()

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelModeMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
