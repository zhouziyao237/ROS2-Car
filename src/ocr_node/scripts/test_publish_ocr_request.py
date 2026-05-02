#!/usr/bin/env python3
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestOCRRequestPub(Node):
    def __init__(self):
        super().__init__('test_ocr_request_pub')
        self.pub = self.create_publisher(String, '/vision/ocr_requests', 10)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.done = False

    def publish_once(self):
        if self.done:
            return
        bbox = [int(x) for x in sys.argv[1:5]] if len(sys.argv) >= 5 else [0, 0, 640, 480]
        data = {
            'stamp': str(time.time()),
            'requests': [
                {
                    'target_id': 'manual_0',
                    'bbox': bbox,
                }
            ]
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)
        self.get_logger().info(f'Published OCR request: {msg.data}')
        self.done = True


def main():
    rclpy.init()
    node = TestOCRRequestPub()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
