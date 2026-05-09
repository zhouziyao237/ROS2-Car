#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from target_pose_msgs.msg import TargetPoseInfo

# Your OAK driver currently publishes /yolov5/detections as yolo_msg/msg/YoloDetectionArray.
# This node subscribes to that type directly.
from yolo_msg.msg import YoloDetectionArray


class ParsedDetection:
    def __init__(self, cx, cy, w, h, score=1.0, label=''):
        self.cx = float(cx)
        self.cy = float(cy)
        self.w = float(w)
        self.h = float(h)
        self.score = float(score)
        self.label = str(label) if label is not None else ''


class TargetPositionOverlayNode(Node):
    """Single-camera demo node for OAK + yolo_msg detections.

    Inputs:
      - RGB image
      - yolo_msg/msg/YoloDetectionArray
      - OAK depth image
      - CameraInfo
      - TF from camera optical frame to target_frame

    Outputs:
      - /target_position_overlay/image
      - /target_pose_base (frame_id is the configured target_frame)
      - /target_pose_info_base
    """

    def __init__(self):
        super().__init__('target_position_overlay_node')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('detections_topic', '/yolov5/detections')
        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('annotated_topic', '/target_position_overlay/image')
        self.declare_parameter('target_pose_topic', '/target_pose_base')
        self.declare_parameter('target_info_topic', '/target_pose_info_base')
        self.declare_parameter('target_frame', 'oak_camera_link')
        self.declare_parameter('min_depth_mm', 150.0)
        self.declare_parameter('max_depth_mm', 5000.0)
        self.declare_parameter('roi_half_size', 8)
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('draw_all_boxes', True)
        self.declare_parameter('debug_log', True)
        self.declare_parameter('assume_normalized_bbox', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.target_info_topic = self.get_parameter('target_info_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_depth_mm = float(self.get_parameter('min_depth_mm').value)
        self.max_depth_mm = float(self.get_parameter('max_depth_mm').value)
        self.roi_half_size = int(self.get_parameter('roi_half_size').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.draw_all_boxes = bool(self.get_parameter('draw_all_boxes').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)
        self.assume_normalized_bbox = bool(self.get_parameter('assume_normalized_bbox').value)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_header = None
        self.latest_depth = None
        self.camera_info = None
        self.latest_detections = []
        self.last_warn_time = self.get_clock().now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(YoloDetectionArray, self.detections_topic, self.detections_callback, 10)

        self.image_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.pose_pub = self.create_publisher(PointStamped, self.target_pose_topic, 10)
        self.info_pub = self.create_publisher(TargetPoseInfo, self.target_info_topic, 10)

        self.timer = self.create_timer(0.05, self.publish_overlay)

        self.get_logger().info('target_position_overlay_node started with yolo_msg/msg/YoloDetectionArray input')
        self.get_logger().info(f'image_topic={self.image_topic}')
        self.get_logger().info(f'detections_topic={self.detections_topic}')
        self.get_logger().info(f'depth_topic={self.depth_topic}')
        self.get_logger().info(f'camera_info_topic={self.camera_info_topic}')
        self.get_logger().info(f'target_frame={self.target_frame}')

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_header = msg.header
        except Exception as e:
            self.get_logger().warn(f'Failed to convert RGB image: {e}')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def detections_callback(self, msg: YoloDetectionArray):
        self.latest_detections = self.parse_yolo_array(msg)
        if self.debug_log and len(self.latest_detections) > 0:
            best = max(self.latest_detections, key=lambda d: d.score)
            self.get_logger().info(
                f'parsed detections={len(self.latest_detections)}, '
                f'best cx={best.cx:.1f} cy={best.cy:.1f} w={best.w:.1f} h={best.h:.1f} score={best.score:.3f}'
            )

    def publish_overlay(self):
        if self.latest_image is None:
            return

        image = self.latest_image.copy()
        if self.camera_info is None or self.latest_depth is None:
            self.draw_status(image, 'Waiting for depth/camera_info')
            self.publish_image(image)
            return

        if not self.latest_detections:
            self.draw_status(image, 'Waiting for yolo_msg detections')
            self.publish_image(image)
            return

        det = self.select_best_detection(self.latest_detections)
        if det is None or det.score < self.min_score:
            self.draw_status(image, 'No valid target')
            self.publish_image(image)
            return

        result = self.compute_target(det)
        if result is None:
            self.draw_bbox(image, det, (0, 0, 255))
            self.draw_status(image, 'Target found, invalid depth/TF')
            self.publish_image(image)
            return

        camera_point, target_point, info = result
        self.pose_pub.publish(target_point)
        self.info_pub.publish(info)

        if self.draw_all_boxes:
            for d in self.latest_detections:
                self.draw_bbox(image, d, (120, 120, 120))

        self.draw_bbox(image, det, (0, 255, 0))
        self.draw_target_text(image, det, camera_point, target_point, info)
        self.publish_image(image)

        if self.debug_log:
            self.get_logger().info(
                f'target {self.target_frame}: '
                f'x={target_point.point.x*1000.0:.1f} '
                f'y={target_point.point.y*1000.0:.1f} '
                f'z={target_point.point.z*1000.0:.1f} mm'
            )

    def select_best_detection(self, detections):
        if not detections:
            return None
        return max(detections, key=lambda d: d.score)

    # ---------- yolo_msg parser ----------
    def parse_yolo_array(self, msg):
        raw = self.extract_detection_list(msg)
        parsed = []
        img_h, img_w = self.get_image_shape()
        for det in raw:
            p = self.parse_one_detection(det, img_w, img_h)
            if p is not None and p.w > 1.0 and p.h > 1.0:
                parsed.append(p)
        return parsed

    def extract_detection_list(self, msg):
        # Common array field names.
        for name in ['detections', 'detection', 'results', 'objects', 'boxes', 'targets']:
            if hasattr(msg, name):
                value = getattr(msg, name)
                if isinstance(value, (list, tuple)):
                    return value
        # Fallback: first list-like field in the ROS message.
        for name in getattr(msg, '__slots__', []):
            value = getattr(msg, name)
            if isinstance(value, (list, tuple)):
                return value
        return []

    def parse_one_detection(self, det, img_w, img_h):
        score = self.extract_score(det)
        label = self.extract_label(det)

        # 1) Direct xmin/ymin/xmax/ymax variants.
        xyxy = self.try_get_xyxy(det)
        if xyxy is not None:
            x1, y1, x2, y2 = xyxy
            return self.make_parsed_from_xyxy(x1, y1, x2, y2, score, label, img_w, img_h)

        # 2) Direct center/size variants.
        cxcywh = self.try_get_cxcywh(det)
        if cxcywh is not None:
            cx, cy, w, h = cxcywh
            return self.make_parsed_from_cxcywh(cx, cy, w, h, score, label, img_w, img_h)

        # 3) Nested bbox object.
        if hasattr(det, 'bbox'):
            bbox = getattr(det, 'bbox')
            xyxy = self.try_get_xyxy(bbox)
            if xyxy is not None:
                x1, y1, x2, y2 = xyxy
                return self.make_parsed_from_xyxy(x1, y1, x2, y2, score, label, img_w, img_h)
            cxcywh = self.try_get_cxcywh(bbox)
            if cxcywh is not None:
                cx, cy, w, h = cxcywh
                return self.make_parsed_from_cxcywh(cx, cy, w, h, score, label, img_w, img_h)

        # 3.5) Nested box object for yolo_msg/msg/YoloDetection.
        # Current yolo_msg uses:
        #   det.box.x1, det.box.y1, det.box.x2, det.box.y2
        #   det.box.center_u, det.box.center_v, det.box.width, det.box.height
        if hasattr(det, 'box'):
            box_obj = getattr(det, 'box')

            xyxy = self.try_get_xyxy(box_obj)
            if xyxy is not None:
                x1, y1, x2, y2 = xyxy
                return self.make_parsed_from_xyxy(x1, y1, x2, y2, score, label, img_w, img_h)

            cxcywh = self.try_get_cxcywh(box_obj)
            if cxcywh is not None:
                cx, cy, w, h = cxcywh
                return self.make_parsed_from_cxcywh(cx, cy, w, h, score, label, img_w, img_h)

        # 4) bbox as float array/list.
        for name in ['bbox', 'box', 'xyxy', 'rect']:
            if hasattr(det, name):
                v = getattr(det, name)
                if isinstance(v, (list, tuple)) and len(v) >= 4:
                    x1, y1, x2, y2 = [float(v[i]) for i in range(4)]
                    return self.make_parsed_from_xyxy(x1, y1, x2, y2, score, label, img_w, img_h)

        self.throttled_warn('Cannot parse bbox from yolo_msg detection. Run: ros2 interface show yolo_msg/msg/YoloDetection')
        return None

    def try_get_xyxy(self, obj):
        variants = [
            ('xmin', 'ymin', 'xmax', 'ymax'),
            ('x_min', 'y_min', 'x_max', 'y_max'),
            ('left', 'top', 'right', 'bottom'),
            ('x1', 'y1', 'x2', 'y2'),
            ('x_min_px', 'y_min_px', 'x_max_px', 'y_max_px'),
            ('xmin_px', 'ymin_px', 'xmax_px', 'ymax_px'),
        ]
        for keys in variants:
            if all(hasattr(obj, k) for k in keys):
                return tuple(float(getattr(obj, k)) for k in keys)
        return None

    def try_get_cxcywh(self, obj):
        variants = [
            ('center_x', 'center_y', 'width', 'height'),
            ('cx', 'cy', 'w', 'h'),
            ('x_center', 'y_center', 'width', 'height'),
            ('center_u', 'center_v', 'width', 'height'),
            ('u', 'v', 'width', 'height'),
        ]
        for keys in variants:
            if all(hasattr(obj, k) for k in keys):
                return tuple(float(getattr(obj, k)) for k in keys)

        # vision-like bbox.center.position.x, bbox.size_x if nested object is passed.
        if hasattr(obj, 'center') and hasattr(obj, 'size_x') and hasattr(obj, 'size_y'):
            center = getattr(obj, 'center')
            if hasattr(center, 'position'):
                pos = getattr(center, 'position')
                if hasattr(pos, 'x') and hasattr(pos, 'y'):
                    return float(pos.x), float(pos.y), float(obj.size_x), float(obj.size_y)
        return None

    def extract_score(self, det):
        for name in ['score', 'confidence', 'conf', 'probability', 'prob', 'objectness']:
            if hasattr(det, name):
                return float(getattr(det, name))
        if hasattr(det, 'hypothesis') and hasattr(det.hypothesis, 'score'):
            return float(det.hypothesis.score)
        return 1.0

    def extract_label(self, det):
        for name in ['class_name', 'label', 'name', 'class_id', 'id']:
            if hasattr(det, name):
                return str(getattr(det, name))
        return ''

    def make_parsed_from_xyxy(self, x1, y1, x2, y2, score, label, img_w, img_h):
        x1, y1, x2, y2 = self.denormalize_xyxy(x1, y1, x2, y2, img_w, img_h)
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        return ParsedDetection((x1 + x2) * 0.5, (y1 + y2) * 0.5, x2 - x1, y2 - y1, score, label)

    def make_parsed_from_cxcywh(self, cx, cy, w, h, score, label, img_w, img_h):
        cx, cy, w, h = self.denormalize_cxcywh(cx, cy, w, h, img_w, img_h)
        return ParsedDetection(cx, cy, w, h, score, label)

    def denormalize_xyxy(self, x1, y1, x2, y2, img_w, img_h):
        if self.assume_normalized_bbox or max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 1.5:
            return x1 * img_w, y1 * img_h, x2 * img_w, y2 * img_h
        return x1, y1, x2, y2

    def denormalize_cxcywh(self, cx, cy, w, h, img_w, img_h):
        if self.assume_normalized_bbox or max(abs(cx), abs(cy), abs(w), abs(h)) <= 1.5:
            return cx * img_w, cy * img_h, w * img_w, h * img_h
        return cx, cy, w, h

    def get_image_shape(self):
        if self.latest_image is not None:
            return self.latest_image.shape[0], self.latest_image.shape[1]
        return 640, 640

    def throttled_warn(self, text):
        now = self.get_clock().now()
        if (now - self.last_warn_time).nanoseconds > 2_000_000_000:
            self.get_logger().warn(text)
            self.last_warn_time = now

    # ---------- target geometry ----------
    def compute_target(self, det):
        u = float(det.cx)
        v = float(det.cy)
        z_mm = self.get_depth_median_mm(int(round(u)), int(round(v)))
        if z_mm is None:
            return None

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx0 = float(self.camera_info.k[2])
        cy0 = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None

        x_mm = (u - cx0) * z_mm / fx
        y_mm = (v - cy0) * z_mm / fy
        dist_mm = math.sqrt(x_mm*x_mm + y_mm*y_mm + z_mm*z_mm)
        width_mm = float(det.w) * z_mm / fx
        height_mm = float(det.h) * z_mm / fy

        camera_frame = self.camera_info.header.frame_id or 'oak_rgb_camera_optical_frame'

        camera_point = PointStamped()
        camera_point.header.stamp = self.get_clock().now().to_msg()
        camera_point.header.frame_id = camera_frame
        camera_point.point.x = x_mm / 1000.0
        camera_point.point.y = y_mm / 1000.0
        camera_point.point.z = z_mm / 1000.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            target_point = do_transform_point(camera_point, transform)
            target_point.header.stamp = self.get_clock().now().to_msg()
            target_point.header.frame_id = self.target_frame
        except Exception as e:
            self.throttled_warn(f'TF failed: {camera_point.header.frame_id} -> {self.target_frame}: {e}')
            return None

        info = TargetPoseInfo()
        info.header = target_point.header
        info.valid = True
        info.score = float(det.score)
        info.bbox_center_u = float(u)
        info.bbox_center_v = float(v)
        info.bbox_width_px = float(det.w)
        info.bbox_height_px = float(det.h)
        info.camera_point = camera_point.point
        info.target_point = target_point.point
        info.target_frame = self.target_frame
        info.depth_m = z_mm / 1000.0
        info.distance_3d_m = dist_mm / 1000.0
        info.width_m = width_mm / 1000.0
        info.height_m = height_mm / 1000.0
        info.target_id = det.label
        info.ocr_text = ''
        return camera_point, target_point, info

    def get_depth_median_mm(self, u, v):
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

    # ---------- drawing ----------
    def draw_bbox(self, image, det, color):
        cx = int(round(det.cx))
        cy = int(round(det.cy))
        bw = int(round(det.w))
        bh = int(round(det.h))
        x1 = max(0, cx - bw // 2)
        y1 = max(0, cy - bh // 2)
        x2 = min(image.shape[1] - 1, cx + bw // 2)
        y2 = min(image.shape[0] - 1, cy + bh // 2)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.circle(image, (cx, cy), 4, color, -1)
        if det.label:
            cv2.putText(image, det.label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def draw_status(self, image, text):
        cv2.putText(image, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(image, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    def draw_target_text(self, image, det, camera_point, target_point, info):
        cx = int(round(det.cx))
        cy = int(round(det.cy))
        lines = [
            f'CAM optical: X={camera_point.point.x*1000:.0f} Y={camera_point.point.y*1000:.0f} Z={camera_point.point.z*1000:.0f} mm',
            f'{self.target_frame}: X={target_point.point.x*1000:.0f} Y={target_point.point.y*1000:.0f} Z={target_point.point.z*1000:.0f} mm',
            f'Depth={info.depth_m*1000:.0f} mm Dist={info.distance_3d_m*1000:.0f} mm Score={det.score:.2f}',
        ]
        x = max(10, min(cx + 10, image.shape[1] - 560))
        y = max(30, cy - 55)
        for i, line in enumerate(lines):
            yy = y + i * 28
            cv2.putText(image, line, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4)
            cv2.putText(image, line, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

    def publish_image(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        if self.latest_image_header is not None:
            msg.header = self.latest_image_header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
