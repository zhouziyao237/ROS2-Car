#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockPublisher(Node):
    def __init__(self, topic: str):
        super().__init__('mock_datamatrix_result')
        self.pub = self.create_publisher(String, topic, 10)


def main():
    parser = argparse.ArgumentParser(description='Mock /oak/datamatrix_3d_result_json for testing oak_package_search.')
    parser.add_argument('code', help='Detected code, e.g. S03-L2-0317')
    parser.add_argument('--topic', default='/oak/datamatrix_3d_result_json')
    parser.add_argument('--repeat', type=int, default=10)
    parser.add_argument('--interval', type=float, default=0.15)
    args = parser.parse_args()

    rclpy.init()
    node = MockPublisher(args.topic)

    payload = {
        'count': 1,
        'results': [
            {
                'code': args.code,
                'frame_id': 'oak_camera_link',
                'camera_xyz_mm': [120.0, -30.0, 850.0],
                'depth_mm': 850.0,
                'bbox': [100, 100, 80, 80],
            }
        ],
    }
    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)

    for i in range(max(1, args.repeat)):
        node.pub.publish(msg)
        print(f"[MOCK {i+1}/{max(1,args.repeat)}] topic={args.topic} code={args.code}")
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(max(0.0, args.interval))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
