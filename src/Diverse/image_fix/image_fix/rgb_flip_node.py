import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def parse_reliability(value: str) -> ReliabilityPolicy:
    value = str(value or "").lower().strip()

    if value in ["best_effort", "besteffort", "best-effort", "sensor"]:
        return ReliabilityPolicy.BEST_EFFORT

    if value in ["reliable", "default"]:
        return ReliabilityPolicy.RELIABLE

    return ReliabilityPolicy.BEST_EFFORT


class RgbFlipNode(Node):
    """
    Subscribe raw RGB image and publish flipped/unmirrored RGB image.

    Important QoS design:
    - Astra/OpenNI camera image topics are usually BEST_EFFORT.
    - Some consumers such as YOLO/OCR may use normal reliable subscriptions.
    - Therefore this node subscribes with BEST_EFFORT by default,
      and publishes with RELIABLE by default for better compatibility.
    """

    def __init__(self):
        super().__init__("rgb_flip_node")

        self.declare_parameter("enable_flip", True)
        self.declare_parameter("input_topic", "/camera/rgb/image_raw")
        self.declare_parameter("output_topic", "/camera/rgb/image_unmirrored")

        # OpenCV flipCode:
        #  1  = horizontal flip
        #  0  = vertical flip
        # -1  = horizontal + vertical flip
        self.declare_parameter("flip_code", 1)

        # QoS parameters
        self.declare_parameter("input_reliability", "best_effort")
        self.declare_parameter("output_reliability", "reliable")
        self.declare_parameter("qos_depth", 1)

        self.enable_flip = bool(self.get_parameter("enable_flip").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.flip_code = int(self.get_parameter("flip_code").value)

        self.input_reliability = str(self.get_parameter("input_reliability").value)
        self.output_reliability = str(self.get_parameter("output_reliability").value)
        self.qos_depth = int(self.get_parameter("qos_depth").value)

        self.bridge = CvBridge()

        input_qos = QoSProfile(
            reliability=parse_reliability(self.input_reliability),
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
            durability=DurabilityPolicy.VOLATILE,
        )

        output_qos = QoSProfile(
            reliability=parse_reliability(self.output_reliability),
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            input_qos,
        )

        self.pub = self.create_publisher(
            Image,
            self.output_topic,
            output_qos,
        )

        self.get_logger().info("rgb_flip_node started.")
        self.get_logger().info(f"enable_flip={self.enable_flip}")
        self.get_logger().info(f"input_topic={self.input_topic}")
        self.get_logger().info(f"output_topic={self.output_topic}")
        self.get_logger().info(f"flip_code={self.flip_code}")
        self.get_logger().info(f"input_reliability={self.input_reliability}")
        self.get_logger().info(f"output_reliability={self.output_reliability}")
        self.get_logger().info(f"qos_depth={self.qos_depth}")

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge imgmsg_to_cv2 failed: {e}")
            return

        if self.enable_flip:
            cv_img = cv2.flip(cv_img, self.flip_code)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge cv2_to_imgmsg failed: {e}")
            return

        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RgbFlipNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()