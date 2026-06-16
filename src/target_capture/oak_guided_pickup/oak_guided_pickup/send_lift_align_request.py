#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RequestSender(Node):
    def __init__(self, args):
        super().__init__('send_lift_align_request')
        self.pub = self.create_publisher(String, args.topic, 10)
        self.payload = {
            'command': 'lift_align',
            'pickup_code': args.pickup_code,
            'shelf_id': args.shelf_id,
            'layer_id': args.layer_id,
            'package_code': args.package_code,
            'timeout_sec': args.timeout_sec,
            'request_id': args.request_id or f'manual_lift_align_{int(time.time()*1000)}',
        }
        self.sent = False
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self):
        if self.sent:
            rclpy.shutdown()
            return
        msg = String()
        msg.data = json.dumps(self.payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(f'published request: {msg.data}')
        self.sent = True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/target_capture/oak_guided_pickup/request')
    parser.add_argument('--pickup-code', default='S03-L2-0311')
    parser.add_argument('--shelf-id', default='S03')
    parser.add_argument('--layer-id', default='L2')
    parser.add_argument('--package-code', default='0311')
    parser.add_argument('--timeout-sec', type=float, default=20.0)
    parser.add_argument('--request-id', default='')
    ns = parser.parse_args(args=args)

    rclpy.init()
    node = RequestSender(ns)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
