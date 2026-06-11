#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def fmt_rad_deg(v):
    if v is None:
        return "None"
    try:
        v = float(v)
        return f"{v:+.4f} rad / {math.degrees(v):+.2f} deg"
    except Exception:
        return str(v)


class OakYawErrorWatcher(Node):
    def __init__(self):
        super().__init__("oak_yaw_error_watcher")
        self.sub = self.create_subscription(
            String,
            "/target_detection/package_search/status",
            self.cb,
            10,
        )
        self.get_logger().info("Watching /target_detection/package_search/status ...")

    def cb(self, msg: String):
        raw = msg.data

        try:
            data = json.loads(raw)
        except Exception:
            print(raw)
            return

        status = data.get("status", "")
        message = data.get("message", "")

        target = data.get("target_oak_yaw", None)
        current = data.get("current_oak_yaw", None)
        err = data.get("yaw_error", None)

        # 有些版本可能嵌套在 detail / data 里
        for key in ["detail", "data", "extra"]:
            if isinstance(data.get(key), dict):
                sub = data[key]
                target = sub.get("target_oak_yaw", target)
                current = sub.get("current_oak_yaw", current)
                err = sub.get("yaw_error", err)

        print("")
        print("========== OAK YAW ERROR ==========")
        print(f"status:          {status}")
        print(f"message:         {message}")
        print(f"target_oak_yaw:  {fmt_rad_deg(target)}")
        print(f"current_oak_yaw: {fmt_rad_deg(current)}")
        print(f"yaw_error:       {fmt_rad_deg(err)}")
        print("===================================")


def main():
    rclpy.init()
    node = OakYawErrorWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
