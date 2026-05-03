import math
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from target_matcher_msg.msg import MatchedTarget


class MatchedDepthLocatorNode(Node):
    def __init__(self):
        super().__init__("matched_depth_locator_node")

        self.declare_parameter("matched_target_topic", "/target_matcher/matched_target")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/rgb/camera_info")

        self.declare_parameter("point_topic", "/target_matcher/matched_point")
        self.declare_parameter("text_topic", "/target_matcher/matched_depth_text")

        # RGB 用 /camera/rgb/image_unmirrored，而 depth 通常还是原图坐标，所以默认 true
        self.declare_parameter("mirror_x_for_depth", True)

        self.declare_parameter("depth_window_radius", 4)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 5.0)

        # 超过这个时间没有新的 matched_target，就不再发布旧深度点
        self.declare_parameter("stale_timeout_sec", 0.8)

        self.matched_target_topic = str(self.get_parameter("matched_target_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        self.point_topic = str(self.get_parameter("point_topic").value)
        self.text_topic = str(self.get_parameter("text_topic").value)

        self.mirror_x_for_depth = bool(self.get_parameter("mirror_x_for_depth").value)
        self.depth_window_radius = int(self.get_parameter("depth_window_radius").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_depth_header = None

        self.latest_info = None

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
            self.depth_topic,
            self.on_depth,
            qos_profile_sensor_data,
        )

        # 关键：camera_info 也用 sensor_data QoS，避免 Astra 发布端 best_effort 时收不到
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )

        self.point_pub = self.create_publisher(
            PointStamped,
            self.point_topic,
            10,
        )

        self.text_pub = self.create_publisher(
            String,
            self.text_topic,
            10,
        )

        self.get_logger().info("matched_depth_locator_node started.")
        self.get_logger().info(f"matched_target_topic={self.matched_target_topic}")
        self.get_logger().info(f"depth_topic={self.depth_topic}")
        self.get_logger().info(f"camera_info_topic={self.camera_info_topic}")
        self.get_logger().info(f"mirror_x_for_depth={self.mirror_x_for_depth}")
        self.get_logger().info(f"stale_timeout_sec={self.stale_timeout_sec}")

    def on_camera_info(self, msg: CameraInfo):
        self.latest_info = msg

    def on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert depth image: {exc}")
            return

        self.latest_depth = depth
        self.latest_depth_header = msg.header

        if self.latest_matched is not None:
            self.try_publish_point()

    def on_matched_target(self, msg: MatchedTarget):
        self.latest_matched = msg
        self.latest_matched_time = self.get_clock().now().nanoseconds * 1e-9
        self.try_publish_point()

    def latest_is_stale(self):
        if self.latest_matched is None:
            return True

        now = self.get_clock().now().nanoseconds * 1e-9

        if now - self.latest_matched_time > self.stale_timeout_sec:
            return True

        return False

    def try_publish_point(self):
        if self.latest_is_stale():
            return

        if not bool(self.latest_matched.matched):
            return

        if self.latest_depth is None:
            return

        if self.latest_info is None:
            return

        depth = self.latest_depth
        h, w = depth.shape[:2]

        box = self.latest_matched.box

        cx = int(box.center_u)
        cy = int(box.center_v)

        if self.mirror_x_for_depth:
            cx = (w - 1) - cx

        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))

        z = self.get_median_depth_m(depth, cx, cy)

        if z is None:
            self.get_logger().warn(
                f"No valid depth around matched target: u={cx}, v={cy}"
            )
            return

        fx = float(self.latest_info.k[0])
        fy = float(self.latest_info.k[4])
        ppx = float(self.latest_info.k[2])
        ppy = float(self.latest_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn("Invalid camera intrinsics.")
            return

        x = (float(cx) - ppx) * z / fx
        y = (float(cy) - ppy) * z / fy

        point_msg = PointStamped()
        point_msg.header = self.latest_depth_header

        if self.latest_info.header.frame_id:
            point_msg.header.frame_id = self.latest_info.header.frame_id
        else:
            point_msg.header.frame_id = "camera_rgb_optical_frame"

        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = float(z)

        self.point_pub.publish(point_msg)

        text_msg = String()
        text_msg.data = (
            f"matched=True, code={self.latest_matched.target_code}, "
            f"track_id={self.latest_matched.track_id}, "
            f"u={cx}, v={cy}, "
            f"x={x:.3f}m, y={y:.3f}m, z={z:.3f}m"
        )

        self.text_pub.publish(text_msg)
        self.get_logger().info(text_msg.data)

    def get_median_depth_m(self, depth, u: int, v: int):
        h, w = depth.shape[:2]
        r = max(1, self.depth_window_radius)

        x1 = max(0, u - r)
        x2 = min(w, u + r + 1)

        y1 = max(0, v - r)
        y2 = min(h, v + r + 1)

        patch = depth[y1:y2, x1:x2]

        if patch.size == 0:
            return None

        values = patch.astype(np.float32).reshape(-1)
        values = values[np.isfinite(values)]
        values = values[values > 0]

        if values.size == 0:
            return None

        median = float(np.median(values))

        # 16UC1 常见单位是 mm，数值通常大于 20；32FC1 常见单位是 m
        if median > 20.0:
            median = median / 1000.0

        if not math.isfinite(median):
            return None

        if median < self.min_depth_m or median > self.max_depth_m:
            return None

        return median


def main(args=None):
    rclpy.init(args=args)
    node = MatchedDepthLocatorNode()

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