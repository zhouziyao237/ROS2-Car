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
        self.declare_parameter('auto_topic', '/cmd_vel')
        self.declare_parameter('auto_topic_alt', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('default_mode', 'manual')

        self.manual_topic = self.get_parameter('manual_topic').value
        self.manual_topic_alt = self.get_parameter('manual_topic_alt').value
        self.auto_topic = self.get_parameter('auto_topic').value
        self.auto_topic_alt = self.get_parameter('auto_topic_alt').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.mode = str(self.get_parameter('default_mode').value).strip().lower()

        self.manual_msg = Twist()
        self.auto_msg = Twist()
        self.auto_msg_alt = Twist()
        self.last_manual_time = None
        self.last_auto_time = None
        self.last_auto_alt_time = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.create_subscription(Twist, self.manual_topic, self.manual_cb, 10)
        if isinstance(self.manual_topic_alt, str) and self.manual_topic_alt.strip():
            self.create_subscription(Twist, self.manual_topic_alt, self.manual_cb, 10)
        self.create_subscription(Twist, self.auto_topic, self.auto_cb, 10)
        if isinstance(self.auto_topic_alt, str) and self.auto_topic_alt.strip():
            self.create_subscription(Twist, self.auto_topic_alt, self.auto_alt_cb, 10)
        self.create_subscription(String, self.mode_topic, self.mode_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info(
            f'CmdVel Mux started. mode={self.mode}, manual={self.manual_topic}, '
            f'manual_alt={self.manual_topic_alt}, auto={self.auto_topic}, '
            f'auto_alt={self.auto_topic_alt}, out={self.output_topic}'
        )

    def now_sec(self):
        t = self.get_clock().now().nanoseconds
        return t / 1e9

    def is_fresh(self, last_time):
        if last_time is None:
            return False
        return (self.now_sec() - last_time) <= self.cmd_timeout

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

        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f'Switched control mode to: {self.mode}')
            # 模式切换时先发一次零速度，避免残留命令
            self.pub.publish(Twist())

    def timer_cb(self):
        out = Twist()

        if self.mode == 'manual':
            if self.is_fresh(self.last_manual_time):
                out = self.manual_msg
        else:  # auto
            auto_fresh = self.is_fresh(self.last_auto_time)
            auto_alt_fresh = self.is_fresh(self.last_auto_alt_time)

            # Prefer a non-zero command if available to avoid being masked by a zero stream.
            if auto_fresh and self.is_nonzero_twist(self.auto_msg):
                out = self.auto_msg
            elif auto_alt_fresh and self.is_nonzero_twist(self.auto_msg_alt):
                out = self.auto_msg_alt
            elif auto_fresh:
                out = self.auto_msg
            elif auto_alt_fresh:
                out = self.auto_msg_alt

        # 简单防 NaN
        for v in [
            out.linear.x, out.linear.y, out.linear.z,
            out.angular.x, out.angular.y, out.angular.z
        ]:
            if not math.isfinite(v):
                self.get_logger().warn('Non-finite cmd_vel detected, publish zero instead.')
                out = Twist()
                break

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
