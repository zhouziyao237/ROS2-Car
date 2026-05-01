#!/usr/bin/env python3

from collections import deque
import math
import threading

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ros_astra_s_camera_msgs.msg import TargetPixel, TargetPositionMM


class RGBDPositionNode(Node):
    def __init__(self):
        super().__init__("rgbd_position_node")

        self.declare_parameter("rgb_topic", "/camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image")
        self.declare_parameter("camera_info_topic", "/camera/rgb/camera_info")

        self.declare_parameter("target_pixel_topic", "/astra_s/target_pixel")
        self.declare_parameter("target_position_topic", "/astra_s/target_position_mm")
        self.declare_parameter("target_point_topic", "/astra_s/target_point")
        self.declare_parameter("debug_image_topic", "/astra_s/debug_rgbd_image")

        self.declare_parameter("camera_frame", "camera_rgb_optical_frame")
        self.declare_parameter("depth_unit", "m")

        self.declare_parameter("min_depth_mm", 150.0)
        self.declare_parameter("max_depth_mm", 5000.0)
        self.declare_parameter("sample_radius", 3)
        self.declare_parameter("sync_queue_size", 20)
        self.declare_parameter("sync_slop_sec", 0.15)
        self.declare_parameter("target_sync_slop_sec", 0.10)
        self.declare_parameter("target_history_size", 30)

        self.declare_parameter("use_center_when_no_target", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_target_point_m", True)

        self.declare_parameter("depth_scale_correction", 1.0)
        self.declare_parameter("depth_offset_mm", 0.0)

        self.declare_parameter("rgb_to_depth_offset_u", 0.0)
        self.declare_parameter("rgb_to_depth_offset_v", 0.0)
        self.declare_parameter("rgb_to_depth_scale_u", 1.0)
        self.declare_parameter("rgb_to_depth_scale_v", 1.0)

        self.declare_parameter("camera_to_base_x_mm", 0.0)
        self.declare_parameter("camera_to_base_y_mm", 0.0)
        self.declare_parameter("camera_to_base_z_mm", 0.0)

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
        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop_sec = float(self.get_parameter("sync_slop_sec").value)
        self.target_sync_slop_sec = float(self.get_parameter("target_sync_slop_sec").value)
        self.target_history_size = max(1, int(self.get_parameter("target_history_size").value))

        self.use_center_when_no_target = bool(self.get_parameter("use_center_when_no_target").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.publish_target_point_m = bool(self.get_parameter("publish_target_point_m").value)

        self.depth_scale_correction = float(self.get_parameter("depth_scale_correction").value)
        self.depth_offset_mm = float(self.get_parameter("depth_offset_mm").value)

        self.rgb_to_depth_offset_u = float(self.get_parameter("rgb_to_depth_offset_u").value)
        self.rgb_to_depth_offset_v = float(self.get_parameter("rgb_to_depth_offset_v").value)
        self.rgb_to_depth_scale_u = float(self.get_parameter("rgb_to_depth_scale_u").value)
        self.rgb_to_depth_scale_v = float(self.get_parameter("rgb_to_depth_scale_v").value)

        self.camera_to_base_x_mm = float(self.get_parameter("camera_to_base_x_mm").value)
        self.camera_to_base_y_mm = float(self.get_parameter("camera_to_base_y_mm").value)
        self.camera_to_base_z_mm = float(self.get_parameter("camera_to_base_z_mm").value)

        self.bridge = CvBridge()

        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.has_camera_info = False

        self.target_lock = threading.Lock()
        self.target_history = deque(maxlen=self.target_history_size)

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec
        )
        self.sync.registerCallback(self.rgbd_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.target_pixel_sub = self.create_subscription(
            TargetPixel,
            self.target_pixel_topic,
            self.target_pixel_callback,
            10
        )

        self.target_position_pub = self.create_publisher(
            TargetPositionMM,
            self.target_position_topic,
            10
        )

        self.target_point_pub = self.create_publisher(
            PointStamped,
            self.target_point_topic,
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            10
        )

        self.get_logger().info("Astra S RGB-D position node started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"TargetPixel topic: {self.target_pixel_topic}")
        self.get_logger().info(f"TargetPositionMM topic: {self.target_position_topic}")
        self.get_logger().info(f"RViz PointStamped topic: {self.target_point_topic}")

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

    def target_pixel_callback(self, msg: TargetPixel):
        with self.target_lock:
            self.target_history.append({
                "stamp_ns": self.stamp_to_ns(msg.header.stamp),
                "u": float(msg.u),
                "v": float(msg.v),
                "confidence": float(msg.confidence),
                "target_id": msg.target_id,
            })

    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.has_camera_info:
            self.get_logger().warn("Waiting for camera_info...", throttle_duration_sec=2.0)
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB cv_bridge error: {e}")
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth cv_bridge error: {e}")
            return

        matched_target = self.get_target_for_stamp(rgb_msg.header.stamp)
        has_target = matched_target is not None
        if matched_target is not None:
            u_rgb = matched_target["u"]
            v_rgb = matched_target["v"]
            confidence = matched_target["confidence"]
            target_id = matched_target["target_id"]
        else:
            u_rgb = -1.0
            v_rgb = -1.0
            confidence = 0.0
            target_id = "center"

        if (not has_target or u_rgb < 0 or v_rgb < 0) and self.use_center_when_no_target:
            u_rgb = rgb_image.shape[1] / 2.0
            v_rgb = rgb_image.shape[0] / 2.0
            confidence = 1.0
            target_id = "center"

        if u_rgb < 0 or v_rgb < 0:
            self.get_logger().warn("No valid target pixel received.", throttle_duration_sec=2.0)
            return

        u_depth = int(round(u_rgb * self.rgb_to_depth_scale_u + self.rgb_to_depth_offset_u))
        v_depth = int(round(v_rgb * self.rgb_to_depth_scale_v + self.rgb_to_depth_offset_v))

        if u_depth < 0 or u_depth >= depth_image.shape[1] or v_depth < 0 or v_depth >= depth_image.shape[0]:
            self.publish_invalid(rgb_msg, target_id, u_rgb, v_rgb, confidence)
            self.get_logger().warn(
                f"Target out of depth range: rgb=({u_rgb:.1f},{v_rgb:.1f}), depth=({u_depth},{v_depth})"
            )
            return

        raw_z_mm = self.get_median_depth_mm(depth_image, depth_msg.encoding, u_depth, v_depth)

        if not self.is_valid_depth(raw_z_mm):
            self.publish_invalid(rgb_msg, target_id, u_rgb, v_rgb, confidence)
            self.get_logger().warn(f"Invalid depth at ({u_depth},{v_depth})", throttle_duration_sec=1.0)
            return

        z_mm = raw_z_mm * self.depth_scale_correction + self.depth_offset_mm

        x_mm = (u_rgb - self.cx) * z_mm / self.fx
        y_mm = (v_rgb - self.cy) * z_mm / self.fy

        x_mm += self.camera_to_base_x_mm
        y_mm += self.camera_to_base_y_mm
        z_mm += self.camera_to_base_z_mm

        position_msg = TargetPositionMM()
        position_msg.header.stamp = rgb_msg.header.stamp
        position_msg.header.frame_id = self.camera_frame
        position_msg.target_id = target_id
        position_msg.x_mm = float(x_mm)
        position_msg.y_mm = float(y_mm)
        position_msg.z_mm = float(z_mm)
        position_msg.u = float(u_rgb)
        position_msg.v = float(v_rgb)
        position_msg.depth_mm = float(z_mm)
        position_msg.confidence = float(confidence)
        position_msg.valid = True
        self.target_position_pub.publish(position_msg)

        if self.publish_target_point_m:
            point_msg = PointStamped()
            point_msg.header = position_msg.header
            point_msg.point.x = float(x_mm) / 1000.0
            point_msg.point.y = float(y_mm) / 1000.0
            point_msg.point.z = float(z_mm) / 1000.0
            self.target_point_pub.publish(point_msg)

        if self.publish_debug_image:
            self.publish_debug_image_msg(
                rgb_image,
                rgb_msg.header,
                int(round(u_rgb)),
                int(round(v_rgb)),
                x_mm,
                y_mm,
                z_mm,
                target_id
            )

        self.get_logger().info(
            f"target={target_id} rgb=({u_rgb:.1f},{v_rgb:.1f}) depth=({u_depth},{v_depth}) "
            f"xyz=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) mm",
            throttle_duration_sec=1.0
        )

    def publish_invalid(self, rgb_msg, target_id, u, v, confidence):
        msg = TargetPositionMM()
        msg.header.stamp = rgb_msg.header.stamp
        msg.header.frame_id = self.camera_frame
        msg.target_id = target_id
        msg.u = float(u)
        msg.v = float(v)
        msg.confidence = float(confidence)
        msg.valid = False
        self.target_position_pub.publish(msg)

    def read_depth_mm(self, depth_image, encoding: str, u: int, v: int) -> float:
        if u < 0 or u >= depth_image.shape[1] or v < 0 or v >= depth_image.shape[0]:
            return -1.0

        try:
            raw = float(depth_image[v, u])
        except Exception:
            return -1.0

        if not math.isfinite(raw):
            return -1.0

        if encoding == "16UC1":
            if self.depth_unit == "m":
                return raw * 1000.0
            return raw

        if encoding == "32FC1":
            if self.depth_unit == "mm":
                return raw
            return raw * 1000.0

        return -1.0

    def get_median_depth_mm(self, depth_image, encoding: str, u: int, v: int) -> float:
        values = []
        for dy in range(-self.sample_radius, self.sample_radius + 1):
            for dx in range(-self.sample_radius, self.sample_radius + 1):
                d = self.read_depth_mm(depth_image, encoding, u + dx, v + dy)
                if self.is_valid_depth(d):
                    values.append(d)
        if not values:
            return -1.0
        values.sort()
        return values[len(values) // 2]

    def is_valid_depth(self, depth_mm: float) -> bool:
        return math.isfinite(depth_mm) and self.min_depth_mm <= depth_mm <= self.max_depth_mm

    def stamp_to_ns(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def get_target_for_stamp(self, stamp):
        stamp_ns = self.stamp_to_ns(stamp)
        max_delta_ns = int(self.target_sync_slop_sec * 1_000_000_000.0)

        with self.target_lock:
            while len(self.target_history) > 1 and self.target_history[1]["stamp_ns"] <= stamp_ns - max_delta_ns:
                self.target_history.popleft()

            best_target = None
            best_delta_ns = None
            for entry in self.target_history:
                delta_ns = abs(entry["stamp_ns"] - stamp_ns)
                if delta_ns > max_delta_ns:
                    continue
                if best_delta_ns is None or delta_ns < best_delta_ns:
                    best_target = entry
                    best_delta_ns = delta_ns
            return best_target

    def publish_debug_image_msg(self, image, header, u, v, x_mm, y_mm, z_mm, target_id):
        cv2.circle(image, (u, v), 6, (0, 0, 255), -1)
        cv2.circle(image, (u, v), max(1, self.sample_radius), (255, 0, 0), 1)
        text = f"{target_id} X:{x_mm:.0f} Y:{y_mm:.0f} Z:{z_mm:.0f} mm"
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
    node = RGBDPositionNode()
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
