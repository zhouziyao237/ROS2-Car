#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelModeMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mode_mux')

        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('manual_topic_alt', '/red_standard_robot1/cmd_vel')

        self.declare_parameter('auto_topic', '/cmd_vel_smoothed')
        self.declare_parameter('auto_topic_alt', '/cmd_vel')

        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('chassis_mode_topic', '/chassis_control_mode')
        self.declare_parameter('chassis_mode_timeout', 1.0)

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.8)
        self.declare_parameter('default_mode', 'manual')

        # Control channel remap only.
        # ROS:
        #   linear.x = forward
        #   linear.y = left strafe
        #
        # Your chassis:
        #   vx = strafe
        #   vy = forward
        #
        # Therefore:
        #   output.linear.x = input.linear.y
        #   output.linear.y = input.linear.x
        self.declare_parameter('swap_xy', True)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('invert_z', False)

        self.declare_parameter('debug_cmd', True)
        self.declare_parameter('debug_period', 1.0)

        self.manual_topic = str(self.get_parameter('manual_topic').value)
        self.manual_topic_alt = str(self.get_parameter('manual_topic_alt').value)
        self.auto_topic = str(self.get_parameter('auto_topic').value)
        self.auto_topic_alt = str(self.get_parameter('auto_topic_alt').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.chassis_mode_topic = str(self.get_parameter('chassis_mode_topic').value)

        self.chassis_mode_timeout = float(self.get_parameter('chassis_mode_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.requested_mode = str(self.get_parameter('default_mode').value).strip().lower()
        self.mode = self.requested_mode

        self.swap_xy = bool(self.get_parameter('swap_xy').value)
        self.invert_x = bool(self.get_parameter('invert_x').value)
        self.invert_y = bool(self.get_parameter('invert_y').value)
        self.invert_z = bool(self.get_parameter('invert_z').value)

        self.debug_cmd = bool(self.get_parameter('debug_cmd').value)
        self.debug_period = float(self.get_parameter('debug_period').value)
        self.last_debug_time = 0.0

        self.chassis_mode = None
        self.last_chassis_mode_time = None

        self.manual_msg = Twist()
        self.auto_msg = Twist()
        self.auto_msg_alt = Twist()

        self.last_manual_time = None
        self.last_auto_time = None
        self.last_auto_alt_time = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.create_subscription(Twist, self.manual_topic, self.manual_cb, 10)

        if self.manual_topic_alt.strip():
            self.create_subscription(Twist, self.manual_topic_alt, self.manual_cb, 10)

        self.create_subscription(Twist, self.auto_topic, self.auto_cb, 10)

        if self.auto_topic_alt.strip():
            self.create_subscription(Twist, self.auto_topic_alt, self.auto_alt_cb, 10)

        self.create_subscription(String, self.mode_topic, self.mode_cb, 10)

        if self.chassis_mode_topic.strip():
            self.create_subscription(String, self.chassis_mode_topic, self.chassis_mode_cb, 10)

        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.timer_cb)

        self.get_logger().info(
            f'CmdVel Mux started. mode={self.mode}, '
            f'manual={self.manual_topic}, manual_alt={self.manual_topic_alt}, '
            f'auto={self.auto_topic}, auto_alt={self.auto_topic_alt}, '
            f'out={self.output_topic}, mode_topic={self.mode_topic}, '
            f'chassis_mode_topic={self.chassis_mode_topic}, '
            f'swap_xy={self.swap_xy}, invert_x={self.invert_x}, '
            f'invert_y={self.invert_y}, invert_z={self.invert_z}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_fresh(self, last_time):
        if last_time is None:
            return False
        return (self.now_sec() - last_time) <= self.cmd_timeout

    def is_chassis_mode_fresh(self):
        if self.last_chassis_mode_time is None:
            return False
        return (self.now_sec() - self.last_chassis_mode_time) <= self.chassis_mode_timeout

    @staticmethod
    def is_nonzero_twist(msg: Twist, eps: float = 1e-4) -> bool:
        return (
            abs(msg.linear.x) > eps or
            abs(msg.linear.y) > eps or
            abs(msg.linear.z) > eps or
            abs(msg.angular.x) > eps or
            abs(msg.angular.y) > eps or
            abs(msg.angular.z) > eps
        )

    def manual_cb(self, msg: Twist):
        self.manual_msg = msg
        self.last_manual_time = self.now_sec()

    def auto_cb(self, msg: Twist):
        self.auto_msg = msg
        self.last_auto_time = self.now_sec()

    def auto_alt_cb(self, msg: Twist):
        self.auto_msg_alt = msg
        self.last_auto_alt_time = self.now_sec()

    def mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode not in ['manual', 'auto']:
            self.get_logger().warn(f'Ignore invalid mode: {new_mode}')
            return

        if new_mode != self.requested_mode:
            self.requested_mode = new_mode
            self.refresh_effective_mode()

    def chassis_mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode not in ['manual', 'auto']:
            self.get_logger().warn(f'Ignore invalid chassis mode: {new_mode}')
            return

        self.chassis_mode = new_mode
        self.last_chassis_mode_time = self.now_sec()
        self.refresh_effective_mode()

    def get_effective_mode(self):
        chassis_fresh = self.is_chassis_mode_fresh() and self.chassis_mode is not None

        # Hardware chassis mode has priority.
        # If chassis reports auto, mux must switch to auto.
        # If chassis reports manual, mux must switch to manual.
        if chassis_fresh:
            if self.chassis_mode == 'auto':
                return 'auto'
            if self.chassis_mode == 'manual':
                return 'manual'

        return self.requested_mode

    def refresh_effective_mode(self):
        effective_mode = self.get_effective_mode()

        if effective_mode != self.mode:
            self.mode = effective_mode
            self.get_logger().info(
                f'Switched effective control mode to: {self.mode} '
                f'(requested={self.requested_mode}, chassis={self.chassis_mode})'
            )

            # Stop once during mode switch.
            self.pub.publish(Twist())

    def sanitize_twist(self, msg: Twist) -> Twist:
        values = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ]

        for v in values:
            if not math.isfinite(v):
                self.get_logger().warn('Non-finite cmd_vel detected, publish zero instead.')
                return Twist()

        return msg

    def remap_twist(self, msg: Twist) -> Twist:
        msg = self.sanitize_twist(msg)

        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z
        wz = msg.angular.z

        if self.swap_xy:
            x, y = y, x

        if self.invert_x:
            x = -x

        if self.invert_y:
            y = -y

        if self.invert_z:
            wz = -wz

        out = Twist()
        out.linear.x = x
        out.linear.y = y
        out.linear.z = z

        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = wz

        return out

    def select_input(self):
        out = Twist()
        source = 'zero'

        if self.mode == 'manual':
            if self.is_fresh(self.last_manual_time):
                out = self.manual_msg
                source = 'manual'

        else:
            auto_fresh = self.is_fresh(self.last_auto_time)
            auto_alt_fresh = self.is_fresh(self.last_auto_alt_time)

            if auto_fresh and self.is_nonzero_twist(self.auto_msg):
                out = self.auto_msg
                source = self.auto_topic
            elif auto_alt_fresh and self.is_nonzero_twist(self.auto_msg_alt):
                out = self.auto_msg_alt
                source = self.auto_topic_alt
            elif auto_fresh:
                out = self.auto_msg
                source = self.auto_topic
            elif auto_alt_fresh:
                out = self.auto_msg_alt
                source = self.auto_topic_alt

        return source, out

    def debug_print(self, source: str, raw: Twist, out: Twist):
        if not self.debug_cmd:
            return

        now = self.now_sec()
        if now - self.last_debug_time < self.debug_period:
            return

        self.last_debug_time = now

        self.get_logger().info(
            f'cmd_mux mode={self.mode} source={source} '
            f'raw=({raw.linear.x:.3f},{raw.linear.y:.3f},{raw.angular.z:.3f}) '
            f'out=({out.linear.x:.3f},{out.linear.y:.3f},{out.angular.z:.3f}) '
            f'swap_xy={self.swap_xy}'
        )

    def timer_cb(self):
        self.refresh_effective_mode()

        source, raw = self.select_input()
        out = self.remap_twist(raw)

        self.debug_print(source, raw, out)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()