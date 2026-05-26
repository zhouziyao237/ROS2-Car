#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class ControlModeManager(Node):
    """
    High-level mode manager.

    It does not directly drive the robot.
    It only manages global mode and exploration permissions.

    Modes:
      stop:
        disable exploration

      manual:
        disable exploration

      nav:
        disable exploration
        user can send RViz/Nav2 goal manually

      auto:
        allow semantic_task_manager/frontier_explorer workflow
        exploration is controlled by semantic_task_manager via /explore/enable
    """

    def __init__(self):
        super().__init__('control_mode_manager')

        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('mode_status_topic', '/control_mode/status')
        self.declare_parameter('explore_enable_topic', '/explore/enable')
        self.declare_parameter('default_mode', 'stop')
        self.declare_parameter('publish_period_sec', 0.5)

        self.mode_topic = self.get_parameter('mode_topic').value
        self.mode_status_topic = self.get_parameter('mode_status_topic').value
        self.explore_enable_topic = self.get_parameter('explore_enable_topic').value
        self.mode = str(self.get_parameter('default_mode').value).strip().lower()
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self.valid_modes = {'manual', 'nav', 'auto', 'stop'}

        if self.mode not in self.valid_modes:
            self.get_logger().warn(
                f'invalid default_mode="{self.mode}", fallback to stop'
            )
            self.mode = 'stop'

        self.mode_pub = self.create_publisher(String, self.mode_status_topic, 10)
        self.explore_pub = self.create_publisher(Bool, self.explore_enable_topic, 10)

        self.mode_sub = self.create_subscription(
            String,
            self.mode_topic,
            self.cb_mode,
            10
        )

        self.timer = self.create_timer(self.publish_period_sec, self.tick)

        self.get_logger().info(
            f'control_mode_manager started: '
            f'mode_topic={self.mode_topic}, '
            f'mode_status_topic={self.mode_status_topic}, '
            f'explore_enable_topic={self.explore_enable_topic}, '
            f'default_mode={self.mode}'
        )

        self.apply_mode(self.mode)

    def cb_mode(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode not in self.valid_modes:
            self.get_logger().warn(
                f'ignore invalid mode="{msg.data}". '
                f'valid modes={sorted(self.valid_modes)}'
            )
            return

        if new_mode != self.mode:
            self.get_logger().info(f'mode changed: {self.mode} -> {new_mode}')
            self.mode = new_mode
            self.apply_mode(self.mode)

    def apply_mode(self, mode: str):
        """
        The manager only forces exploration off in non-auto modes.
        In auto mode, semantic_task_manager decides when to enable exploration.
        """
        if mode in ('manual', 'nav', 'stop'):
            self.publish_explore(False)

    def publish_explore(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.explore_pub.publish(msg)

    def tick(self):
        status = String()
        status.data = self.mode
        self.mode_pub.publish(status)

        # Keep exploration disabled in safe / non-auto modes.
        if self.mode in ('manual', 'nav', 'stop'):
            self.publish_explore(False)


def main():
    rclpy.init()
    node = ControlModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
