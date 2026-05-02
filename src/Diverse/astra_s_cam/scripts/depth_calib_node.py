#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthCalibNode(Node):
    def __init__(self):
        super().__init__("depth_calib_node")

        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("sample_radius", 10)
        self.declare_parameter("min_depth_mm", 150.0)
        self.declare_parameter("max_depth_mm", 5000.0)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.sample_radius = int(self.get_parameter("sample_radius").value)
        self.min_depth_mm = float(self.get_parameter("min_depth_mm").value)
        self.max_depth_mm = float(self.get_parameter("max_depth_mm").value)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        self.get_logger().info("astra_s_cam depth calibration node started.")
        self.get_logger().info("Put a matte flat board in the image center. Fit: real = raw * scale + offset")

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth cv_bridge error: {e}")
            return

        h, w = depth.shape[:2]
        u = w // 2
        v = h // 2

        values = []
        for dy in range(-self.sample_radius, self.sample_radius + 1):
            for dx in range(-self.sample_radius, self.sample_radius + 1):
                x = u + dx
                y = v + dy
                if x < 0 or x >= w or y < 0 or y >= h:
                    continue

                try:
                    d = float(depth[y, x])
                except Exception:
                    continue

                if msg.encoding == "32FC1":
                    d *= 1000.0

                if math.isfinite(d) and self.min_depth_mm <= d <= self.max_depth_mm:
                    values.append(d)

        if not values:
            self.get_logger().warn("No valid depth in center area.", throttle_duration_sec=1.0)
            return

        values.sort()
        median = values[len(values) // 2]
        self.get_logger().info(
            f"Center median raw depth = {median:.1f} mm. Measure real distance and update depth_scale_correction/depth_offset_mm.",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
