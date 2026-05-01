#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SimCmdVelModeMux(Node):
    def __init__(self):
        super().__init__('sim_cmd_vel_mode_mux')

        self.mode = 'auto'
        self.latest_manual = Twist()
        self.latest_auto = Twist()
        self.latest_search = Twist()

        self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.auto_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_search', self.search_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_selected', 10)
        self.timer = self.create_timer(0.02, self.publish_selected)

        self.get_logger().info('sim_cmd_vel_mode_mux started.')
        self.get_logger().info('manual -> /cmd_vel_manual, auto -> /cmd_vel, search -> /cmd_vel_search, output -> /cmd_vel_selected')

    def mode_callback(self, msg: String):
        if msg.data not in ['manual', 'auto', 'search']:
            self.get_logger().warn(f'Unknown control mode: {msg.data}')
            return
        if self.mode != msg.data:
            self.get_logger().info(f'Control mode changed: {self.mode} -> {msg.data}')
        self.mode = msg.data

    def manual_callback(self, msg: Twist):
        self.latest_manual = msg

    def auto_callback(self, msg: Twist):
        self.latest_auto = msg

    def search_callback(self, msg: Twist):
        self.latest_search = msg

    def publish_selected(self):
        if self.mode == 'manual':
            self.cmd_pub.publish(self.latest_manual)
        elif self.mode == 'search':
            self.cmd_pub.publish(self.latest_search)
        else:
            self.cmd_pub.publish(self.latest_auto)


def main(args=None):
    rclpy.init(args=args)
    node = SimCmdVelModeMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
