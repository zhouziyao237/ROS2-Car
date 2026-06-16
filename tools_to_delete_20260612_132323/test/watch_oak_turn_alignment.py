#!/usr/bin/env python3
import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WatchOakTurnAlignment(Node):
    def __init__(self):
        super().__init__("watch_oak_turn_alignment")

        self.declare_parameter("status_topic", "/target_detection/package_search/status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("oak_camera_frame", "oak-cam-Link")
        self.declare_parameter("oak_forward_axis", "x")
        self.declare_parameter("oak_yaw_offset_deg", 0.0)
        self.declare_parameter("tick_hz", 2.0)

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.oak_camera_frame = str(self.get_parameter("oak_camera_frame").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_oak_yaw: Optional[float] = None
        self.last_status = ""
        self.last_message = ""
        self.last_cmd_wz = 0.0

        self.create_subscription(String, self.status_topic, self.on_status, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)

        period = 1.0 / max(0.5, float(self.get_parameter("tick_hz").value))
        self.create_timer(period, self.on_timer)

        self.get_logger().info("watch_oak_turn_alignment started")
        self.get_logger().info(f"fixed_frame={self.fixed_frame}")
        self.get_logger().info(f"oak_camera_frame={self.oak_camera_frame}")
        self.get_logger().info("axis mode: current_oak_yaw = selected axis yaw of oak-cam-Link")

    def on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        self.last_status = str(data.get("status", ""))
        self.last_message = str(data.get("message", ""))

        if "target_oak_yaw" in data:
            try:
                self.target_oak_yaw = float(data["target_oak_yaw"])
            except Exception:
                pass

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_wz = float(msg.angular.z)

    def get_current_oak_yaw(self):
        axis = str(self.get_parameter("oak_forward_axis").value).strip().lower()
        yaw_offset = math.radians(float(self.get_parameter("oak_yaw_offset_deg").value))

        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.oak_camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            return None, str(e)

        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w

        r00 = 1.0 - 2.0 * (y * y + z * z)
        r10 = 2.0 * (x * y + w * z)

        r01 = 2.0 * (x * y - w * z)
        r11 = 1.0 - 2.0 * (x * x + z * z)

        r02 = 2.0 * (x * z + w * y)
        r12 = 2.0 * (y * z - w * x)

        if axis in ["x", "+x"]:
            vx, vy = r00, r10
        elif axis == "-x":
            vx, vy = -r00, -r10
        elif axis in ["y", "+y"]:
            vx, vy = r01, r11
        elif axis == "-y":
            vx, vy = -r01, -r11
        elif axis in ["z", "+z"]:
            vx, vy = r02, r12
        elif axis == "-z":
            vx, vy = -r02, -r12
        else:
            return None, f"invalid oak_forward_axis={axis}"

        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return None, f"axis={axis} has near-zero projection"

        return normalize_angle(math.atan2(vy, vx) + yaw_offset), None

    def on_timer(self):
        if self.target_oak_yaw is None:
            print("[WAIT] no target_oak_yaw yet. Send package_search request first.")
            return

        current_yaw, err = self.get_current_oak_yaw()
        if err is not None:
            print(f"[TF ERROR] {err}")
            return

        yaw_error = normalize_angle(self.target_oak_yaw - current_yaw)
        axis = str(self.get_parameter("oak_forward_axis").value)

        print(
            "[OAK ALIGN] "
            f"status={self.last_status or '-'} | "
            f"axis={axis} | "
            f"target={self.target_oak_yaw:+.3f} rad ({math.degrees(self.target_oak_yaw):+.1f} deg) | "
            f"current={current_yaw:+.3f} rad ({math.degrees(current_yaw):+.1f} deg) | "
            f"error={yaw_error:+.3f} rad ({math.degrees(yaw_error):+.1f} deg) | "
            f"cmd_wz={self.last_cmd_wz:+.3f} | "
            f"msg={self.last_message}"
        )


def main():
    rclpy.init()
    node = WatchOakTurnAlignment()
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
