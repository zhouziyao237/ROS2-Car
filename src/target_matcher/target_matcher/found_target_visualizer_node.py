#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from target_matcher_msg.msg import MatchedTarget


class FoundTargetVisualizerNode(Node):
    def __init__(self):
        super().__init__("found_target_visualizer_node")

        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("matched_target_topic", "/target_matcher/matched_target")
        self.declare_parameter("output_topic", "/target_matcher/found_target_image")

        self.image_topic = self.get_parameter("image_topic").value
        self.matched_target_topic = self.get_parameter("matched_target_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.bridge = CvBridge()
        self.latest_target = None

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.target_sub = self.create_subscription(
            MatchedTarget,
            self.matched_target_topic,
            self.target_callback,
            10,
        )

        self.image_pub = self.create_publisher(
            Image,
            self.output_topic,
            10,
        )

        self.get_logger().info("found_target_visualizer_node started")
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"matched_target_topic={self.matched_target_topic}")
        self.get_logger().info(f"output_topic={self.output_topic}")

    def target_callback(self, msg: MatchedTarget):
        self.latest_target = msg

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.latest_target is not None and self.latest_target.matched:
            box = self.latest_target.box

            x1 = int(box.x1)
            y1 = int(box.y1)
            x2 = int(box.x2)
            y2 = int(box.y2)

            h, w = frame.shape[:2]

            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))

            if x2 > x1 and y2 > y1:
                target_code = self.latest_target.target_code
                matched_token = self.latest_target.matched_token
                track_id = self.latest_target.track_id
                conf = self.latest_target.yolo_confidence

                label = f"FOUND {target_code} token={matched_token} conf={conf:.2f}"

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3,
                )

                cv2.putText(
                    frame,
                    label,
                    (x1, max(30, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    frame,
                    track_id,
                    (x1, min(h - 10, y2 + 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FoundTargetVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()