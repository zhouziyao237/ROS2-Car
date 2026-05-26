#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from robot_logistics_interfaces.msg import TaskState

class DepthAlignNode(Node):
    def __init__(self):
        super().__init__('depth_align_node')
        self.declare_parameter('cmd_vel_topic', '/align/cmd_vel')
        self.declare_parameter('auto_success_for_sim', True)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.done_pub = self.create_publisher(Bool, '/align/done', 10)
        self.create_subscription(TaskState, '/task/state', self.state_cb, 10)
        self.aligning = False
        self.get_logger().info('depth_align_node started; replace stub with your depth visual servo')
    def state_cb(self, msg):
        if msg.state == TaskState.STATE_ALIGNING and not self.aligning:
            self.aligning = True
            self.get_logger().info('ALIGNING started')
            if bool(self.get_parameter('auto_success_for_sim').value):
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info('sim auto align done')
        elif msg.state != TaskState.STATE_ALIGNING:
            self.aligning = False
            self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args); node = DepthAlignNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
