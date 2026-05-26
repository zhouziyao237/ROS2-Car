#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .semantic_utils import SemanticLibraryStore

class SemanticBusinessMapNode(Node):
    def __init__(self):
        super().__init__('semantic_business_map_node')
        self.declare_parameter('semantic_map_path', '/host-home/maps/semantic_library.json')
        self.declare_parameter('reset_semantic_map', False)
        self.declare_parameter('reset_current_run_only', True)
        self.declare_parameter('status_topic', '/semantic_map/status')
        self.path = self.get_parameter('semantic_map_path').value
        self.reset_all = bool(self.get_parameter('reset_semantic_map').value)
        self.reset_current_run_only = bool(self.get_parameter('reset_current_run_only').value)
        self.run_id = str(int(time.time()))
        self.store = SemanticLibraryStore(self.path, self.run_id)
        if self.reset_all:
            from .semantic_utils import default_library
            self.store.save(default_library(self.run_id))
            self.get_logger().warn('semantic library reset completely')
        elif self.reset_current_run_only:
            self.store.reset_current_run()
            self.get_logger().info('semantic library loaded; current_map_pose reset for this SLAM run')
        else:
            self.store.load()
        self.pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info(f'semantic_business_map_node ready: {self.path}, run_id={self.run_id}')
    def timer_cb(self):
        data = self.store.load(); m = String()
        rel = data.get('relocalization', {})
        m.data = f"shelves={len(data.get('shelves',{}))}, relocalized={rel.get('has_transform', False)}, conf={rel.get('confidence',0.0):.2f}"
        self.pub.publish(m)

def main(args=None):
    rclpy.init(args=args); node = SemanticBusinessMapNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
