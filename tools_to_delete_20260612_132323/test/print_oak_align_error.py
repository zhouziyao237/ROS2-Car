#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def deg(v):
    if v is None:
        return None
    try:
        return math.degrees(float(v))
    except Exception:
        return None


def fmt_rad_deg(v):
    if v is None:
        return "None"
    try:
        v = float(v)
        return f"{v:+.4f} rad / {math.degrees(v):+.2f} deg"
    except Exception:
        return str(v)


class PrintOakAlignError(Node):
    def __init__(self):
        super().__init__("print_oak_align_error")
        self.create_subscription(
            String,
            "/target_detection/package_search/status",
            self.cb,
            10,
        )
        self.get_logger().info("listening /target_detection/package_search/status")

    def cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            print("[RAW]", msg.data)
            return

        status = d.get("status")
        state = d.get("state")
        message = d.get("message")

        target = d.get("target_oak_yaw")
        current = d.get("current_oak_yaw")
        error = d.get("yaw_error")
        cmd_wz = d.get("cmd_wz")

        oak_x = d.get("oak_x")
        oak_y = d.get("oak_y")
        shelf_x = d.get("shelf_x")
        shelf_y = d.get("shelf_y")
        shelf_front_yaw = d.get("shelf_front_yaw")

        target_mode = d.get("target_mode")
        oak_frame = d.get("oak_camera_frame")
        oak_forward_vector = d.get("oak_forward_vector")

        print("=" * 80)
        print(f"status/state : {status} / {state}")
        print(f"message      : {message}")
        print(f"target_mode  : {target_mode}")
        print(f"oak_frame    : {oak_frame}")
        print(f"oak_vector   : {oak_forward_vector}")
        print("-" * 80)
        print(f"target yaw   : {fmt_rad_deg(target)}")
        print(f"current yaw  : {fmt_rad_deg(current)}")
        print(f"yaw error    : {fmt_rad_deg(error)}")
        print(f"cmd_wz       : {cmd_wz}")
        print("-" * 80)
        print(f"oak pos      : x={oak_x}, y={oak_y}")
        print(f"shelf pos    : x={shelf_x}, y={shelf_y}")
        print(f"shelf yaw    : {fmt_rad_deg(shelf_front_yaw)}")

        try:
            e_deg = abs(math.degrees(float(error)))
            if e_deg <= 5.0:
                print("[JUDGE] 已经在 5 deg 容差内，理论上应该 aligned 并停止。")
            else:
                print(f"[JUDGE] 还差 {e_deg:.2f} deg，节点会继续转。")
        except Exception:
            pass


def main():
    rclpy.init()
    node = PrintOakAlignError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
