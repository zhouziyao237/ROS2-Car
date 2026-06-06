#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import re
import sys
import time
import unicodedata

import rclpy
from std_msgs.msg import String


def normalize_pickup_code(text: str) -> str:
    text = unicodedata.normalize("NFKC", str(text)).strip().upper()
    text = re.sub(r"[\s_/]+", "-", text)
    text = re.sub(r"[^A-Z0-9-]+", "-", text)
    text = re.sub(r"-+", "-", text).strip("-")
    return text


def parse_pickup_code(text: str):
    norm = normalize_pickup_code(text)
    m = re.search(r"(^|[^A-Z0-9])([A-Z0-9]{1,4})-(L\d{1,3})-([A-Z0-9]{4})([^A-Z0-9]|$)", norm)
    if not m:
        raise ValueError(
            f"Invalid pickup code: {text}. Expected format like S03-L1-0301"
        )

    shelf_id = m.group(2)
    layer_id = m.group(3)
    package_code = m.group(4)
    pickup_code = f"{shelf_id}-{layer_id}-{package_code}"

    return {
        "pickup_code": pickup_code,
        "shelf_id": shelf_id,
        "layer_id": layer_id,
        "package_code": package_code,
        "cell_id": pickup_code,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Publish a logistics pickup task using a full DataMatrix pickup code, e.g. S03-L1-0301."
    )
    parser.add_argument("pickup_code", help="Full pickup code, e.g. S03-L1-0301")
    parser.add_argument("--topic", default="/task/shelf_goal", help="Task topic. Default: /task/shelf_goal")
    parser.add_argument("--raw", action="store_true", help="Publish raw code string instead of JSON")
    parser.add_argument("--include-parsed", action="store_true", help="Include parsed shelf_id/layer_id/package_code fields in JSON")
    parser.add_argument("--repeat", type=int, default=3, help="Publish repeat count. Default: 3")
    parser.add_argument("--interval", type=float, default=0.2, help="Seconds between repeated publishes. Default: 0.2")
    args = parser.parse_args()

    try:
        parsed = parse_pickup_code(args.pickup_code)
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(2)

    if args.raw:
        payload = parsed["pickup_code"]
    else:
        data = {
            "task_type": "pickup",
            "pickup_code": parsed["pickup_code"],
        }
        if args.include_parsed:
            data.update({
                "shelf_id": parsed["shelf_id"],
                "layer_id": parsed["layer_id"],
                "package_code": parsed["package_code"],
                "cell_id": parsed["cell_id"],
            })
        payload = json.dumps(data, ensure_ascii=False)

    rclpy.init()
    node = rclpy.create_node("send_pickup_task")
    pub = node.create_publisher(String, args.topic, 10)

    # Give DDS discovery a short moment.
    end = time.time() + 0.8
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    msg = String()
    msg.data = payload

    for i in range(max(1, args.repeat)):
        pub.publish(msg)
        print(f"[PUBLISH {i+1}/{max(1, args.repeat)}] topic={args.topic} data={payload}")
        rclpy.spin_once(node, timeout_sec=max(0.01, args.interval))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
