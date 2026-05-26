#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class GrabExecuteNode(Node):
    def __init__(self):
        super().__init__('grab_execute_node')
        self.grab_pub = self.create_publisher(Bool, '/grab/cmd', 10)
        self.create_subscription(Bool, '/align/done', self.done_cb, 10)
        self.get_logger().info('grab_execute_node started')
    def done_cb(self, msg):
        if msg.data:
            self.get_logger().info('align done -> publish /grab/cmd')
            self.grab_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args); node = GrabExecuteNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
