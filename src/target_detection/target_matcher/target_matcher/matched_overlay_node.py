import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from target_matcher_msg.msg import MatchedTarget


class MatchedOverlayNode(Node):
    def __init__(self):
        super().__init__("matched_overlay_node")

        self.declare_parameter("image_topic", "/camera/rgb/image_unmirrored")
        self.declare_parameter("matched_target_topic", "/target_matcher/matched_target")
        self.declare_parameter("output_image_topic", "/target_matcher/matched_overlay_image")

        self.declare_parameter("draw_unmatched", False)
        self.declare_parameter("line_thickness", 3)
        self.declare_parameter("show_text", True)

        # 超过这个时间没有新的 matched_target，就不再画旧框
        self.declare_parameter("stale_timeout_sec", 0.8)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.matched_target_topic = str(self.get_parameter("matched_target_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)

        self.draw_unmatched = bool(self.get_parameter("draw_unmatched").value)
        self.line_thickness = int(self.get_parameter("line_thickness").value)
        self.show_text = bool(self.get_parameter("show_text").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        self.bridge = CvBridge()

        self.latest_matched = None
        self.latest_matched_time = 0.0

        self.create_subscription(
            MatchedTarget,
            self.matched_target_topic,
            self.on_matched_target,
            10,
        )

        self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            Image,
            self.output_image_topic,
            10,
        )

        self.get_logger().info("matched_overlay_node started.")
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"matched_target_topic={self.matched_target_topic}")
        self.get_logger().info(f"output_image_topic={self.output_image_topic}")
        self.get_logger().info(f"stale_timeout_sec={self.stale_timeout_sec}")

    def on_matched_target(self, msg: MatchedTarget):
        self.latest_matched = msg
        self.latest_matched_time = self.get_clock().now().nanoseconds * 1e-9

    def latest_is_stale(self):
        if self.latest_matched is None:
            return True

        now = self.get_clock().now().nanoseconds * 1e-9

        if now - self.latest_matched_time > self.stale_timeout_sec:
            return True

        return False

    def on_image(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        draw = image.copy()

        if not self.latest_is_stale():
            matched = bool(self.latest_matched.matched)

            if matched or self.draw_unmatched:
                box = self.latest_matched.box

                x1 = max(0, min(int(box.x1), draw.shape[1] - 1))
                y1 = max(0, min(int(box.y1), draw.shape[0] - 1))
                x2 = max(0, min(int(box.x2), draw.shape[1] - 1))
                y2 = max(0, min(int(box.y2), draw.shape[0] - 1))

                if x2 > x1 and y2 > y1:
                    color = (0, 0, 255) if matched else (0, 255, 255)
                    cv2.rectangle(draw, (x1, y1), (x2, y2), color, self.line_thickness)

                    if self.show_text:
                        if matched:
                            label = f"MATCH {self.latest_matched.target_code}"

                            if self.latest_matched.matched_token:
                                label += f" [{self.latest_matched.matched_token}]"
                        else:
                            label = "NO MATCH"

                        text_y = max(20, y1 - 8)

                        cv2.putText(
                            draw,
                            label,
                            (x1, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            color,
                            2,
                            cv2.LINE_AA,
                        )

        out = self.bridge.cv2_to_imgmsg(draw, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MatchedOverlayNode()

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