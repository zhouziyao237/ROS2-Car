#!/usr/bin/env python3

import math
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


class RGBDLocatorNode(Node):
    def __init__(self):
        super().__init__("rgbd_locator_node")

        self.declare_parameter("rgb_topic", "/camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/rgb/camera_info")

        self.declare_parameter("target_pixel_topic", "/astra_s_cam/target_pixel")
        self.declare_parameter("target_position_topic", "/astra_s_cam/target_position_mm")
        self.declare_parameter("target_point_topic", "/astra_s_cam/target_point")
        self.declare_parameter("debug_image_topic", "/astra_s_cam/debug_image")

        self.declare_parameter("camera_frame", "camera_rgb_optical_frame")
        self.declare_parameter("depth_unit", "mm")

        self.declare_parameter("min_depth_mm", 150.0)
        self.declare_parameter("max_depth_mm", 5000.0)
        self.declare_parameter("sample_radius", 3)

        self.declare_parameter("use_center_when_no_target", False)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("depth_scale_correction", 1.0)
        self.declare_parameter("depth_offset_mm", 0.0)

        self.declare_parameter("rgb_to_depth_offset_u", 0.0)
        self.declare_parameter("rgb_to_depth_offset_v", 0.0)
        self.declare_parameter("rgb_to_depth_scale_u", 1.0)
        self.declare_parameter("rgb_to_depth_scale_v", 1.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        self.target_pixel_topic = self.get_parameter("target_pixel_topic").value
        self.target_position_topic = self.get_parameter("target_position_topic").value
        self.target_point_topic = self.get_parameter("target_point_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.camera_frame = self.get_parameter("camera_frame").value
        self.depth_unit = self.get_parameter("depth_unit").value

        self.min_depth_mm = float(self.get_parameter("min_depth_mm").value)
        self.max_depth_mm = float(self.get_parameter("max_depth_mm").value)
        self.sample_radius = int(self.get_parameter("sample_radius").value)

        self.use_center_when_no_target = bool(
            self.get_parameter("use_center_when_no_target").value
        )
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.depth_scale_correction = float(
            self.get_parameter("depth_scale_correction").value
        )
        self.depth_offset_mm = float(
            self.get_parameter("depth_offset_mm").value
        )

        self.rgb_to_depth_offset_u = float(
            self.get_parameter("rgb_to_depth_offset_u").value
        )
        self.rgb_to_depth_offset_v = float(
            self.get_parameter("rgb_to_depth_offset_v").value
        )
        self.rgb_to_depth_scale_u = float(
            self.get_parameter("rgb_to_depth_scale_u").value
        )
        self.rgb_to_depth_scale_v = float(
            self.get_parameter("rgb_to_depth_scale_v").value
        )

        self.bridge = CvBridge()

        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.has_camera_info = False

        self.target_lock = threading.Lock()
        self.has_target_pixel = False
        self.target_u = -1.0
        self.target_v = -1.0
        self.target_confidence = 0.0

        # Astra S image stream uses sensor-data QoS / Best Effort.
        # RViz2 and this locator must match it, otherwise DDS will not deliver messages.
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.normal_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.rgb_sub = Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=self.sensor_qos
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=self.sensor_qos
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=20,
            slop=0.15
        )
        self.sync.registerCallback(self.rgbd_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            self.sensor_qos
        )

        self.target_pixel_sub = self.create_subscription(
            PointStamped,
            self.target_pixel_topic,
            self.target_pixel_callback,
            self.normal_qos
        )

        self.target_position_pub = self.create_publisher(
            PointStamped,
            self.target_position_topic,
            self.normal_qos
        )

        self.target_point_pub = self.create_publisher(
            PointStamped,
            self.target_point_topic,
            self.normal_qos
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            self.sensor_qos
        )

        self.get_logger().info("astra_s_cam RGB-D locator started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"CameraInfo topic: {self.camera_info_topic}")
        self.get_logger().info(f"Target pixel topic: {self.target_pixel_topic}")
        self.get_logger().info(f"Target position mm topic: {self.target_position_topic}")
        self.get_logger().info(f"RViz point topic: {self.target_point_topic}")
        self.get_logger().info("Image subscription QoS: Best Effort")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

        if not self.has_camera_info:
            self.get_logger().info(
                f"CameraInfo received: fx={self.fx:.3f}, fy={self.fy:.3f}, "
                f"cx={self.cx:.3f}, cy={self.cy:.3f}, frame={self.camera_frame}"
            )

        self.has_camera_info = True

    def target_pixel_callback(self, msg: PointStamped):
        with self.target_lock:
            self.target_u = float(msg.point.x)
            self.target_v = float(msg.point.y)
            self.target_confidence = float(msg.point.z)
            self.has_target_pixel = True

    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.has_camera_info:
            self.get_logger().warn(
                "Waiting for camera_info...",
                throttle_duration_sec=2.0
            )
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"RGB cv_bridge error: {e}")
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Depth cv_bridge error: {e}")
            return

        with self.target_lock:
            has_target = self.has_target_pixel
            u_rgb = self.target_u
            v_rgb = self.target_v
            confidence = self.target_confidence

        if (not has_target or u_rgb < 0.0 or v_rgb < 0.0):
            if self.use_center_when_no_target:
                u_rgb = rgb_image.shape[1] / 2.0
                v_rgb = rgb_image.shape[0] / 2.0
                confidence = 1.0
            else:
                self.get_logger().warn(
                    "No valid target pixel received.",
                    throttle_duration_sec=2.0
                )
                return

        u_depth = int(round(
            u_rgb * self.rgb_to_depth_scale_u + self.rgb_to_depth_offset_u
        ))
        v_depth = int(round(
            v_rgb * self.rgb_to_depth_scale_v + self.rgb_to_depth_offset_v
        ))

        if (
            u_depth < 0
            or v_depth < 0
            or u_depth >= depth_image.shape[1]
            or v_depth >= depth_image.shape[0]
        ):
            self.get_logger().warn(
                f"Target pixel out of depth image range: "
                f"rgb=({u_rgb:.1f},{v_rgb:.1f}), "
                f"depth=({u_depth},{v_depth})"
            )
            return

        z_mm_raw = self.get_median_depth_mm(
            depth_image,
            depth_msg.encoding,
            u_depth,
            v_depth
        )

        if not self.is_valid_depth(z_mm_raw):
            self.get_logger().warn(
                f"Invalid depth at depth pixel ({u_depth},{v_depth})",
                throttle_duration_sec=1.0
            )
            return

        z_mm = z_mm_raw * self.depth_scale_correction + self.depth_offset_mm

        if self.fx <= 0.0 or self.fy <= 0.0:
            self.get_logger().warn("Invalid camera intrinsics.")
            return

        x_mm = (u_rgb - self.cx) * z_mm / self.fx
        y_mm = (v_rgb - self.cy) * z_mm / self.fy

        # Output for algorithm/control: PointStamped in mm.
        out_mm = PointStamped()
        out_mm.header.stamp = rgb_msg.header.stamp
        out_mm.header.frame_id = self.camera_frame
        out_mm.point.x = float(x_mm)
        out_mm.point.y = float(y_mm)
        out_mm.point.z = float(z_mm)
        self.target_position_pub.publish(out_mm)

        # Output for RViz2: PointStamped in meters.
        out_m = PointStamped()
        out_m.header.stamp = rgb_msg.header.stamp
        out_m.header.frame_id = self.camera_frame
        out_m.point.x = float(x_mm / 1000.0)
        out_m.point.y = float(y_mm / 1000.0)
        out_m.point.z = float(z_mm / 1000.0)
        self.target_point_pub.publish(out_m)

        if self.publish_debug_image:
            self.publish_debug_image_msg(
                rgb_image,
                rgb_msg.header,
                int(round(u_rgb)),
                int(round(v_rgb)),
                x_mm,
                y_mm,
                z_mm
            )

        self.get_logger().info(
            f"target rgb=({u_rgb:.1f},{v_rgb:.1f}), "
            f"depth=({u_depth},{v_depth}), "
            f"xyz=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) mm, "
            f"confidence={confidence:.2f}",
            throttle_duration_sec=1.0
        )

    def read_depth_mm(self, depth_image, encoding: str, u: int, v: int) -> float:
        if (
            u < 0
            or v < 0
            or u >= depth_image.shape[1]
            or v >= depth_image.shape[0]
        ):
            return -1.0

        try:
            raw = float(depth_image[v, u])
        except Exception:
            return -1.0

        if not math.isfinite(raw):
            return -1.0

        if encoding == "16UC1":
            if self.depth_unit == "mm":
                return raw
            if self.depth_unit == "m":
                return raw * 1000.0
            return raw

        if encoding == "32FC1":
            if self.depth_unit == "m":
                return raw * 1000.0
            if self.depth_unit == "mm":
                return raw
            return raw * 1000.0

        # Some OpenNI drivers may publish mono16-like depth encodings.
        if "16" in encoding:
            return raw

        return -1.0

    def get_median_depth_mm(self, depth_image, encoding: str, u: int, v: int) -> float:
        values = []

        for dy in range(-self.sample_radius, self.sample_radius + 1):
            for dx in range(-self.sample_radius, self.sample_radius + 1):
                d = self.read_depth_mm(
                    depth_image,
                    encoding,
                    u + dx,
                    v + dy
                )

                if self.is_valid_depth(d):
                    values.append(d)

        if not values:
            return -1.0

        values.sort()
        return values[len(values) // 2]

    def is_valid_depth(self, depth_mm: float) -> bool:
        return (
            math.isfinite(depth_mm)
            and self.min_depth_mm <= depth_mm <= self.max_depth_mm
        )

    def publish_debug_image_msg(self, image, header, u, v, x_mm, y_mm, z_mm):
        cv2.circle(image, (u, v), 6, (0, 0, 255), -1)
        cv2.circle(image, (u, v), max(2, self.sample_radius), (255, 0, 0), 1)

        text = f"X:{x_mm:.0f} Y:{y_mm:.0f} Z:{z_mm:.0f} mm"

        cv2.putText(
            image,
            text,
            (u + 10, max(20, v - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header = header
        self.debug_image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = RGBDLocatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()