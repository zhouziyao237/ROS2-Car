#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from ros_astra_s_camera_msgs.msg import TargetPixel


class TargetPixelDemoNode(Node):
    def __init__(self):
        super().__init__("target_pixel_demo_node")

        self.declare_parameter("rgb_topic", "/camera/rgb/image_raw")
        self.declare_parameter("target_pixel_topic", "/astra_s/target_pixel")
        self.declare_parameter("debug_image_topic", "/astra_s/detector_debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("target_id", "red_blob")
        self.declare_parameter("min_area", 1200.0)

        # Default HSV range roughly matches a red object. The detector is only
        # a demo bridge; replace it with your own model output in real use.
        self.declare_parameter("lower_hsv_1", [0, 120, 70])
        self.declare_parameter("upper_hsv_1", [10, 255, 255])
        self.declare_parameter("lower_hsv_2", [170, 120, 70])
        self.declare_parameter("upper_hsv_2", [180, 255, 255])

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.target_pixel_topic = self.get_parameter("target_pixel_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.target_id = str(self.get_parameter("target_id").value)
        self.min_area = float(self.get_parameter("min_area").value)

        self.lower_hsv_1 = np.array(self.get_parameter("lower_hsv_1").value, dtype=np.uint8)
        self.upper_hsv_1 = np.array(self.get_parameter("upper_hsv_1").value, dtype=np.uint8)
        self.lower_hsv_2 = np.array(self.get_parameter("lower_hsv_2").value, dtype=np.uint8)
        self.upper_hsv_2 = np.array(self.get_parameter("upper_hsv_2").value, dtype=np.uint8)

        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.target_pub = self.create_publisher(TargetPixel, self.target_pixel_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info("Target pixel demo node started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"TargetPixel topic: {self.target_pixel_topic}")

    def rgb_callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB cv_bridge error: {e}")
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_1 = cv2.inRange(hsv, self.lower_hsv_1, self.upper_hsv_1)
        mask_2 = cv2.inRange(hsv, self.lower_hsv_2, self.upper_hsv_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return

        x, y, w, h = cv2.boundingRect(contour)
        u = x + w / 2.0
        v = y + h / 2.0

        target_msg = TargetPixel()
        target_msg.header = msg.header
        target_msg.u = float(u)
        target_msg.v = float(v)
        target_msg.confidence = min(1.0, area / max(self.min_area * 4.0, 1.0))
        target_msg.target_id = self.target_id
        self.target_pub.publish(target_msg)

        if self.publish_debug_image:
            debug = image.copy()
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, (int(round(u)), int(round(v))), 5, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f"{self.target_id} ({u:.0f}, {v:.0f})",
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPixelDemoNode()
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
