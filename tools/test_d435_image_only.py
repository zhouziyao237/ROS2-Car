#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class TestD435ImageOnly(Node):
    def __init__(self):
        super().__init__("test_d435_image_only")

        self.bridge = CvBridge()
        self.image_count = 0
        self.info_printed = False

        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.on_image,
            10,
        )

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.on_info,
            10,
        )

        self.get_logger().info("test_d435_image_only started")

    def on_info(self, msg):
        if not self.info_printed:
            self.info_printed = True
            self.get_logger().info(
                f"camera_info ok: frame={msg.header.frame_id}, "
                f"width={msg.width}, height={msg.height}, "
                f"k0={msg.k[0]:.2f}, k2={msg.k[2]:.2f}, k4={msg.k[4]:.2f}, k5={msg.k[5]:.2f}"
            )

    def on_image(self, msg):
        self.image_count += 1

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.image_count == 1 or self.image_count % 30 == 0:
            self.get_logger().info(
                f"image ok: count={self.image_count}, "
                f"shape={img.shape}, encoding={msg.encoding}, "
                f"frame={msg.header.frame_id}"
            )


def main():
    rclpy.init()
    node = TestD435ImageOnly()

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
