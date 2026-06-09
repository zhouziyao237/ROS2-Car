#!/usr/bin/env python3
import argparse
import json
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def normalize_pickup_code(s: str) -> str:
    raw = str(s).strip().upper().replace('_', '-').replace('/', '-')
    raw = re.sub(r"\s+", "-", raw)
    raw = re.sub(r"[^A-Z0-9-]+", "-", raw)
    raw = re.sub(r"-+", "-", raw).strip('-')
    return raw


def parse_pickup_code(code: str):
    norm = normalize_pickup_code(code)
    m = re.fullmatch(r"([A-Z0-9]{1,4})-(L[0-9]{1,3})-([A-Z0-9]{4})", norm)
    if not m:
        raise SystemExit(f"invalid pickup_code: {code}; expected S03-L2-0317")
    return norm, m.group(1), m.group(2), m.group(3)


class Sender(Node):
    def __init__(self, topic: str):
        super().__init__('send_package_search_request')
        self.pub = self.create_publisher(String, topic, 10)


def main():
    parser = argparse.ArgumentParser(description='Publish OAK package search request.')
    parser.add_argument('pickup_code', help='Full pickup code, e.g. S03-L2-0317')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--target-oak-yaw', type=float, help='Target OAK yaw in rad. Used directly.')
    group.add_argument('--approach-yaw', type=float, help='Navigation approach yaw in rad. Used as target OAK yaw.')
    group.add_argument('--shelf-front-yaw', type=float, help='Shelf front outward normal yaw in rad. Node will add pi.')

    parser.add_argument('--topic', default='/target_detection/package_search/request')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--interval', type=float, default=0.2)
    args = parser.parse_args()

    pickup_code, shelf_id, layer_id, package_code = parse_pickup_code(args.pickup_code)

    payload = {
        'task_type': 'package_search',
        'pickup_code': pickup_code,
        'shelf_id': shelf_id,
        'layer_id': layer_id,
        'package_code': package_code,
    }
    if args.target_oak_yaw is not None:
        payload['target_oak_yaw'] = float(args.target_oak_yaw)
    elif args.approach_yaw is not None:
        payload['approach_yaw'] = float(args.approach_yaw)
    elif args.shelf_front_yaw is not None:
        payload['shelf_front_yaw'] = float(args.shelf_front_yaw)

    rclpy.init()
    node = Sender(args.topic)
    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False)

    # Allow discovery.
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)

    for i in range(max(1, args.repeat)):
        node.pub.publish(msg)
        print(f"[PUBLISH {i+1}/{max(1,args.repeat)}] topic={args.topic} data={msg.data}")
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(max(0.0, args.interval))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
