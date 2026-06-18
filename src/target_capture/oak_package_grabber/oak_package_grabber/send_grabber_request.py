#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Sender(Node):
    def __init__(self, topic: str):
        super().__init__('send_grabber_request')
        self.pub = self.create_publisher(String, topic, 10)


def main():
    parser = argparse.ArgumentParser(description='Send oak_package_grabber request.')
    parser.add_argument('command', choices=[
        'grab_pickup', 'open_gripper', 'close_gripper', 'extend_slide', 'extend_screw', 'retract_all'
    ])
    parser.add_argument('--topic', default='/target_capture/oak_package_grabber/request')
    parser.add_argument('--request-id', default='')
    parser.add_argument('--pickup-code', default='S03-L2-0312')
    parser.add_argument('--package-code', default='0312')
    parser.add_argument('--shelf-id', default='S03')
    parser.add_argument('--layer-id', default='L2')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--slide', type=float, default=None, help='target_slide_x_mm for extend_slide')
    parser.add_argument('--screw', type=float, default=None, help='target_screw_x_mm for extend_screw')
    args = parser.parse_args()

    rclpy.init()
    node = Sender(args.topic)

    req = {
        'command': args.command,
        'request_id': args.request_id or f'debug_{args.command}_{int(time.time() * 1000)}',
        'pickup_code': args.pickup_code,
        'package_code': args.package_code,
        'shelf_id': args.shelf_id,
        'layer_id': args.layer_id,
        'timeout_sec': args.timeout_sec,
        'source': 'send_grabber_request',
    }
    if args.slide is not None:
        req['target_slide_x_mm'] = args.slide
    if args.screw is not None:
        req['target_screw_x_mm'] = args.screw

    msg = String()
    msg.data = json.dumps(req, ensure_ascii=False)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.pub.publish(msg)
    print(msg.data)
    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
