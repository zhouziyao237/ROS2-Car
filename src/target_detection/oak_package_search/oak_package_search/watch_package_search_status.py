#!/usr/bin/env python3
import argparse
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Watcher(Node):
    def __init__(self, topic: str):
        super().__init__('watch_package_search_status')
        self.sub = self.create_subscription(String, topic, self.cb, 10)

    def cb(self, msg: String):
        try:
            obj = json.loads(msg.data)
            print(json.dumps(obj, ensure_ascii=False, indent=2))
        except Exception:
            print(msg.data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/target_detection/package_search/status')
    args = parser.parse_args()

    rclpy.init()
    node = Watcher(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
