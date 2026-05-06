#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


class Target3DFusionNode(Node):
    def __init__(self):
        super().__init__('target_3d_fusion_node')

        self.declare_parameter('detections_topic', '/target_matcher/matched_target')
        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('target_pose_camera_topic', '/target_pose_camera')
        self.declare_parameter('target_size_camera_topic', '/target_size_camera')
        self.declare_parameter('min_depth_mm', 150.0)
        self.declare_parameter('max_depth_mm', 5000.0)
        self.declare_parameter('roi_half_size', 8)
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('debug_log', True)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pose_topic = self.get_parameter('target_pose_camera_topic').value
        self.size_topic = self.get_parameter('target_size_camera_topic').value
        self.min_depth_mm = float(self.get_parameter('min_depth_mm').value)
        self.max_depth_mm = float(self.get_parameter('max_depth_mm').value)
        self.roi_half_size = int(self.get_parameter('roi_half_size').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_header = None
        self.camera_info = None

        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Detection2DArray, self.detections_topic, self.detections_callback, 10)

        self.pose_pub = self.create_publisher(PointStamped, self.pose_topic, 10)
        self.size_pub = self.create_publisher(Vector3Stamped, self.size_topic, 10)

        self.get_logger().info('target_3d_fusion_node started')
        self.get_logger().info(f'detections_topic={self.detections_topic}')
        self.get_logger().info(f'depth_topic={self.depth_topic}')
        self.get_logger().info(f'camera_info_topic={self.camera_info_topic}')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def detections_callback(self, msg: Detection2DArray):
        if self.latest_depth is None or self.camera_info is None:
            return
        if not msg.detections:
            return

        det, score = self.select_best_detection(msg)
        if det is None or score < self.min_score:
            return

        u = float(det.bbox.center.position.x)
        v = float(det.bbox.center.position.y)
        depth_mm = self.get_depth_median_mm(int(round(u)), int(round(v)))
        if depth_mm is None:
            if self.debug_log:
                self.get_logger().warn(f'No valid depth at target u={u:.1f}, v={v:.1f}')
            return

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn('Invalid camera intrinsics fx/fy')
            return

        x_mm = (u - cx) * depth_mm / fx
        y_mm = (v - cy) * depth_mm / fy
        z_mm = depth_mm

        bbox_w_px = float(det.bbox.size_x)
        bbox_h_px = float(det.bbox.size_y)
        width_mm = bbox_w_px * z_mm / fx
        height_mm = bbox_h_px * z_mm / fy
        distance_mm = math.sqrt(x_mm * x_mm + y_mm * y_mm + z_mm * z_mm)

        camera_frame = self.camera_info.header.frame_id
        if camera_frame == '':
            camera_frame = msg.header.frame_id

        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = camera_frame
        p.point.x = x_mm / 1000.0
        p.point.y = y_mm / 1000.0
        p.point.z = z_mm / 1000.0
        self.pose_pub.publish(p)

        s = Vector3Stamped()
        s.header = p.header
        s.vector.x = width_mm / 1000.0
        s.vector.y = height_mm / 1000.0
        s.vector.z = distance_mm / 1000.0
        self.size_pub.publish(s)

        if self.debug_log:
            self.get_logger().info(
                f'camera target: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm, '
                f'size={width_mm:.1f}x{height_mm:.1f} mm, frame={camera_frame}'
            )

    def select_best_detection(self, msg: Detection2DArray):
        best = None
        best_score = -1.0
        for det in msg.detections:
            score = 1.0
            if det.results:
                score = float(det.results[0].hypothesis.score)
            if score > best_score:
                best = det
                best_score = score
        return best, best_score

    def get_depth_median_mm(self, u: int, v: int):
        depth = self.latest_depth
        if depth is None:
            return None
        h, w = depth.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None
        r = self.roi_half_size
        x1, x2 = max(0, u - r), min(w, u + r + 1)
        y1, y2 = max(0, v - r), min(h, v + r + 1)
        roi = depth[y1:y2, x1:x2].astype(np.float32)
        valid = roi[np.isfinite(roi)]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None
        med = float(np.median(valid))
        if med < 20.0:
            valid = valid * 1000.0
        valid = valid[(valid >= self.min_depth_mm) & (valid <= self.max_depth_mm)]
        if valid.size == 0:
            return None
        return float(np.median(valid))


def main(args=None):
    rclpy.init(args=args)
    node = Target3DFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
