#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelModeMux(Node):
    """
    Select manual or auto cmd_vel and output to /cmd_vel_selected.

    ROS standard:
      linear.x  > 0 : forward
      linear.y  > 0 : left
      angular.z > 0 : turn left / counter-clockwise

    Parameters:
      manual_topic
      manual_topic_alt / manual_alt_topic
      auto_topic
      auto_topic_alt
      nav_topic
      nav_alt_topic
      output_topic
      mode_topic
      default_mode
      swap_xy
      invert_x
      invert_y
      invert_z
      debug_cmd
      debug_period
      cmd_timeout / cmd_timeout_sec
      publish_rate
    """

    def __init__(self):
        super().__init__('cmd_vel_mode_mux')

        # Topics
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('manual_topic_alt', '')
        self.declare_parameter('manual_alt_topic', '')

        self.declare_parameter('auto_topic', '/cmd_vel_smoothed')
        self.declare_parameter('auto_topic_alt', '/cmd_vel')
        self.declare_parameter('nav_topic', '/cmd_vel')
        self.declare_parameter('nav_alt_topic', '/cmd_vel_smoothed')

        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('chassis_mode_topic', '')

        # Mode / timing
        self.declare_parameter('default_mode', 'auto')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.08)
        self.declare_parameter('cmd_timeout_sec', 0.08)

        # Axis transform
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('invert_z', False)

        # Debug
        self.declare_parameter('debug_cmd', False)
        self.declare_parameter('debug_period', 0.5)

        self.manual_topic = self.get_parameter('manual_topic').value
        self.manual_topic_alt = self.get_parameter('manual_topic_alt').value
        self.manual_alt_topic = self.get_parameter('manual_alt_topic').value

        self.auto_topic = self.get_parameter('auto_topic').value
        self.auto_topic_alt = self.get_parameter('auto_topic_alt').value
        self.nav_topic = self.get_parameter('nav_topic').value
        self.nav_alt_topic = self.get_parameter('nav_alt_topic').value

        self.output_topic = self.get_parameter('output_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.chassis_mode_topic = self.get_parameter('chassis_mode_topic').value

        self.mode = str(self.get_parameter('default_mode').value).lower()
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # Use cmd_timeout_sec first; keep cmd_timeout for compatibility.
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        if self.cmd_timeout <= 0.0:
            self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.swap_xy = bool(self.get_parameter('swap_xy').value)
        self.invert_x = bool(self.get_parameter('invert_x').value)
        self.invert_y = bool(self.get_parameter('invert_y').value)
        self.invert_z = bool(self.get_parameter('invert_z').value)

        self.debug_cmd = bool(self.get_parameter('debug_cmd').value)
        self.debug_period = float(self.get_parameter('debug_period').value)

        self.last_manual_msg = Twist()
        self.last_auto_msg = Twist()
        self.last_manual_time = 0.0
        self.last_auto_time = 0.0
        self.last_debug_time = 0.0

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.create_subscription(String, self.mode_topic, self.on_mode, 10)
        if self.chassis_mode_topic:
            self.create_subscription(String, self.chassis_mode_topic, self.on_mode, 10)

        self.create_subscription(Twist, self.manual_topic, self.on_manual_cmd, 10)
        if self.manual_topic_alt:
            self.create_subscription(Twist, self.manual_topic_alt, self.on_manual_cmd, 10)
        if self.manual_alt_topic and self.manual_alt_topic != self.manual_topic_alt:
            self.create_subscription(Twist, self.manual_alt_topic, self.on_manual_cmd, 10)

        self.create_subscription(Twist, self.auto_topic, self.on_auto_cmd, 10)
        if self.auto_topic_alt:
            self.create_subscription(Twist, self.auto_topic_alt, self.on_auto_cmd, 10)
        if self.nav_topic and self.nav_topic not in [self.auto_topic, self.auto_topic_alt]:
            self.create_subscription(Twist, self.nav_topic, self.on_auto_cmd, 10)
        if self.nav_alt_topic and self.nav_alt_topic not in [self.auto_topic, self.auto_topic_alt, self.nav_topic]:
            self.create_subscription(Twist, self.nav_alt_topic, self.on_auto_cmd, 10)

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            'cmd_vel_mode_mux started: '
            f'mode_topic={self.mode_topic}, '
            f'manual_topic={self.manual_topic}, '
            f'auto_topic={self.auto_topic}, '
            f'nav_topic={self.nav_topic}, '
            f'output_topic={self.output_topic}, '
            f'default_mode={self.mode}, '
            f'swap_xy={self.swap_xy}, '
            f'invert_x={self.invert_x}, '
            f'invert_y={self.invert_y}, '
            f'invert_z={self.invert_z}, '
            f'debug_cmd={self.debug_cmd}'
        )

    def on_mode(self, msg: String):
        m = msg.data.strip().lower()

        if m in ['manual', 'teleop', 'keyboard']:
            self.mode = 'manual'
        elif m in ['auto', 'nav', 'navigation']:
            self.mode = 'auto'
        elif m in ['stop', 'idle', 'zero']:
            self.mode = 'stop'
        else:
            self.get_logger().warn(f'Unknown control mode: {msg.data}')

    def on_manual_cmd(self, msg: Twist):
        self.last_manual_msg = msg
        self.last_manual_time = time.monotonic()

    def on_auto_cmd(self, msg: Twist):
        self.last_auto_msg = msg
        self.last_auto_time = time.monotonic()

    def zero_twist(self):
        return Twist()

    def transform_cmd(self, msg: Twist):
        out = Twist()

        x = float(msg.linear.x)
        y = float(msg.linear.y)
        z = float(msg.angular.z)

        if self.swap_xy:
            x, y = y, x

        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y
        if self.invert_z:
            z = -z

        out.linear.x = x
        out.linear.y = y
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = z

        return out

    def select_raw_cmd(self):
        now = time.monotonic()

        if self.mode == 'manual':
            if now - self.last_manual_time <= self.cmd_timeout:
                return self.last_manual_msg, 'manual'
            return self.zero_twist(), 'manual_timeout'

        if self.mode == 'auto':
            if now - self.last_auto_time <= self.cmd_timeout:
                return self.last_auto_msg, 'auto'
            return self.zero_twist(), 'auto_timeout'

        return self.zero_twist(), 'stop'

    def on_timer(self):
        raw, source = self.select_raw_cmd()
        out = self.transform_cmd(raw)
        self.pub.publish(out)

        if self.debug_cmd:
            now = time.monotonic()
            if now - self.last_debug_time >= self.debug_period:
                self.last_debug_time = now
                self.get_logger().info(
                    f'[cmd_mux] mode={self.mode}, source={source}, '
                    f'raw vx={raw.linear.x:.3f}, vy={raw.linear.y:.3f}, wz={raw.angular.z:.3f} -> '
                    f'out vx={out.linear.x:.3f}, vy={out.linear.y:.3f}, wz={out.angular.z:.3f}; '
                    f'swap_xy={self.swap_xy}, '
                    f'invert_x={self.invert_x}, '
                    f'invert_y={self.invert_y}, '
                    f'invert_z={self.invert_z}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
