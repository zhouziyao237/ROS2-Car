#!/usr/bin/env python3
import argparse
import json
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


PICKUP_RE = re.compile(r"^([A-Z0-9]{1,4})-(L[0-9]{1,3})-([A-Z0-9]{4})$")


def normalize_pickup_code(text: str) -> str:
    text = str(text).strip().upper()
    text = text.replace("_", "-").replace("/", "-")
    text = re.sub(r"\s+", "-", text)
    text = re.sub(r"-+", "-", text)
    return text


def parse_pickup_code(text: str):
    code = normalize_pickup_code(text)
    m = PICKUP_RE.match(code)
    if not m:
        raise ValueError(
            f"Invalid pickup code: {text}. Expected format: S03-L2-0317"
        )

    shelf_id, layer_id, package_code = m.group(1), m.group(2), m.group(3)

    return {
        "task_type": "package_search",
        "pickup_code": code,
        "shelf_id": shelf_id,
        "layer_id": layer_id,
        "package_code": package_code,
        "cell_id": code,
    }


class OakPackageSearchBTTest(Node):
    def __init__(self, args):
        super().__init__("oak_package_search_bt_test")
        self.args = args
        self.request_topic = args.request_topic
        self.status_topic = args.status_topic

        self.pub = self.create_publisher(String, self.request_topic, 10)
        self.sub = self.create_subscription(
            String,
            self.status_topic,
            self.status_cb,
            20,
        )

        self.last_status = None
        self.done = False
        self.exit_code = 3

    def status_cb(self, msg: String):
        raw = msg.data
        print(f"[STATUS RAW] {raw}", flush=True)

        try:
            obj = json.loads(raw)
        except Exception:
            return

        status = str(obj.get("status", "")).lower()
        state = str(obj.get("state", "")).lower()
        message = str(obj.get("message", ""))

        self.last_status = obj

        if status == "found" or state == "found":
            print("[RESULT] FOUND", flush=True)
            self.done = True
            self.exit_code = 0

        elif status in ["failed", "failure", "canceled", "cancelled"] or state in ["failed", "failure", "canceled", "cancelled"]:
            print(f"[RESULT] FAILED/CANCELED: {message}", flush=True)
            self.done = True
            self.exit_code = 2

    def build_request(self):
        req = parse_pickup_code(self.args.pickup_code)

        if self.args.target_oak_yaw is not None:
            req["target_oak_yaw"] = float(self.args.target_oak_yaw)

        if self.args.shelf_front_yaw is not None:
            req["shelf_front_yaw"] = float(self.args.shelf_front_yaw)

        if self.args.approach_yaw is not None:
            req["approach_yaw"] = float(self.args.approach_yaw)

        req["source"] = "bt_like_test_tool"
        req["stamp"] = time.time()

        return req

    def publish_request_once(self):
        req = self.build_request()
        msg = String()
        msg.data = json.dumps(req, ensure_ascii=False)

        print("============================================================")
        print("[BT-LIKE REQUEST]")
        print(f"topic: {self.request_topic}")
        print(json.dumps(req, ensure_ascii=False, indent=2))
        print("============================================================")

        self.pub.publish(msg)

    def run(self):
        # Let ROS discovery finish.
        start_wait = time.time()
        while time.time() - start_wait < 0.8:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.publish_request_once()

        start = time.time()
        timeout = float(self.args.timeout)

        while rclpy.ok() and not self.done:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start > timeout:
                print(f"[RESULT] TIMEOUT after {timeout:.1f}s", flush=True)
                self.exit_code = 3
                self.done = True
                break

        return self.exit_code


def main():
    parser = argparse.ArgumentParser(
        description="Temporary BT-like tester for RunOakPackageSearch."
    )

    parser.add_argument("pickup_code", help="Full pickup code, e.g. S03-L2-0317")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--target-oak-yaw", type=float, help="Direct target OAK yaw in map frame, rad.")
    group.add_argument("--shelf-front-yaw", type=float, help="Shelf front outward normal yaw, rad. Node will use +pi internally.")
    group.add_argument("--approach-yaw", type=float, help="Navigation approach yaw, rad.")

    parser.add_argument("--request-topic", default="/target_detection/package_search/request")
    parser.add_argument("--status-topic", default="/target_detection/package_search/status")
    parser.add_argument("--timeout", type=float, default=35.0)

    args = parser.parse_args()

    rclpy.init()
    node = OakPackageSearchBTTest(args)

    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    sys.exit(code)


if __name__ == "__main__":
    main()
