#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR + depth perception node for final row-column shelf logic.

Supported QR texts:
  S01-C1       -> column_qr
  S01-L2       -> layer_qr
  PKG:A1234    -> package_qr

Publishes:
  /vision/semantic_observation
  /vision/qr_debug_image
"""

import math
import re
import struct

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
from robot_logistics_interfaces.msg import SemanticObservation

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    HAS_PYZBAR = True
except Exception:
    HAS_PYZBAR = False


class VisionQRDepthNode(Node):
    def __init__(self):
        super().__init__('vision_qr_depth_node')

        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_frame', 'camera_link')
        self.declare_parameter('depth_topics', ['/camera/camera/aligned_depth_to_color/image_raw'])
        self.declare_parameter('points_topics', ['/camera/camera/points', '/camera/points'])
        self.declare_parameter('output_topic', '/vision/semantic_observation')
        self.declare_parameter('debug_image_topic', '/vision/qr_debug_image')
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('depth_window_px', 5)
        self.declare_parameter('publish_debug', True)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.output_topic = self.get_parameter('output_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth_window_px = int(self.get_parameter('depth_window_px').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)

        self.depth_topics = list(self.get_parameter('depth_topics').value)
        self.points_topics = list(self.get_parameter('points_topics').value)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.camera_info = None
        self.last_depth = None
        self.last_depth_topic = ''
        self.last_cloud = None
        self.last_cloud_topic = ''
        self.warn_count = 0

        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos_profile_sensor_data)

        for topic in self.depth_topics:
            self.create_subscription(Image, topic, lambda msg, t=topic: self.depth_cb(msg, t), qos_profile_sensor_data)
        for topic in self.points_topics:
            self.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.cloud_cb(msg, t), qos_profile_sensor_data)

        self.obs_pub = self.create_publisher(SemanticObservation, self.output_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info('vision_qr_depth_node started')
        self.get_logger().info(f'pyzbar_available={HAS_PYZBAR}')
        self.get_logger().info(f'rgb_topic={self.rgb_topic}')
        self.get_logger().info(f'camera_info_topic={self.camera_info_topic}')
        self.get_logger().info(f'depth_topics={self.depth_topics}')
        self.get_logger().info(f'points_topics={self.points_topics}')

    def info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg, topic):
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth_topic = topic
        except Exception as exc:
            self.get_logger().warn(f'failed to convert depth from {topic}: {repr(exc)}')

    def cloud_cb(self, msg, topic):
        self.last_cloud = msg
        self.last_cloud_topic = topic

    def rgb_cb(self, msg):
        if self.camera_info is None:
            self.throttled_warn('waiting for camera_info')
            return
        if self.last_depth is None and self.last_cloud is None:
            self.throttled_warn('waiting for depth image or pointcloud')
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'failed to convert RGB image: {repr(exc)}')
            return

        debug = bgr.copy()
        detections = self.detect_qr(bgr)

        for raw_text, pts in detections:
            text = raw_text.strip()
            parsed = self.parse_qr_text(text)
            if parsed is None:
                self.draw_qr(debug, text, pts, (0, 0, 255))
                continue

            center_u, center_v = self.center_of_pts(pts)
            xyz = self.get_xyz(center_u, center_v)
            if xyz is None:
                self.draw_qr(debug, f'{text} no_xyz', pts, (0, 165, 255))
                continue

            x, y, z, source = xyz
            normal = self.estimate_normal_from_qr_corners(pts)
            nx, ny, nz = normal
            yaw = math.atan2(ny, nx)

            obs = SemanticObservation()
            obs.header.stamp = msg.header.stamp
            obs.header.frame_id = msg.header.frame_id
            obs.obs_type = parsed['obs_type']
            obs.shelf_id = parsed.get('shelf_id', '')
            obs.face = parsed.get('face', '')
            obs.row = int(parsed.get('row', 0))
            obs.col = int(parsed.get('col', 0))
            obs.layer_id = parsed.get('layer_id', '')
            obs.column_id = parsed.get('column_id', '')
            obs.package_code = parsed.get('package_code', '')
            obs.confidence = 0.85
            obs.has_pose = True

            pose = PoseStamped()
            pose.header = obs.header
            pose.header.frame_id = self.output_frame
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.z = float(math.sin(yaw * 0.5))
            pose.pose.orientation.w = float(math.cos(yaw * 0.5))
            obs.pose = pose

            self.obs_pub.publish(obs)
            self.draw_qr(debug, text, pts, (0, 255, 0))

            self.get_logger().info(
                f'QR obs raw="{text}" type={obs.obs_type} shelf={obs.shelf_id} '
                f'layer={obs.layer_id} column={obs.column_id} row={obs.row} col={obs.col} '
                f'pkg={obs.package_code} camera_xyz=({x:.2f},{y:.2f},{z:.2f}) '
                f'normal=({nx:.2f},{ny:.2f},{nz:.2f}) source={source}'
            )

        if self.publish_debug:
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            except Exception:
                pass

    def parse_qr_text(self, text):
        # Column QR: S01-C1
        m = re.fullmatch(r'(S\d+)-C(\d+)', text, flags=re.IGNORECASE)
        if m:
            shelf_id = m.group(1).upper()
            col = int(m.group(2))
            return {
                'obs_type': 'column_qr',
                'shelf_id': shelf_id,
                'column_id': f'C{col}',
                'col': col,
                'row': 0,
                'layer_id': '',
                'package_code': '',
            }

        # Layer QR: S01-L2
        m = re.fullmatch(r'(S\d+)-L(\d+)', text, flags=re.IGNORECASE)
        if m:
            shelf_id = m.group(1).upper()
            row = int(m.group(2))
            return {
                'obs_type': 'layer_qr',
                'shelf_id': shelf_id,
                'layer_id': f'L{row}',
                'row': row,
                'col': 0,
                'column_id': '',
                'package_code': '',
            }

        # Package QR: PKG:A1234
        m = re.fullmatch(r'PKG:([^:]+)', text, flags=re.IGNORECASE)
        if m:
            return {
                'obs_type': 'package_qr',
                'shelf_id': '',
                'row': 0,
                'col': 0,
                'layer_id': '',
                'column_id': '',
                'package_code': m.group(1).strip(),
            }

        return None

    def detect_qr(self, bgr):
        results = []
        if HAS_PYZBAR:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            decoded = pyzbar_decode(gray)
            for d in decoded:
                try:
                    text = d.data.decode('utf-8').strip()
                except Exception:
                    continue
                if not text:
                    continue
                poly = d.polygon
                if poly and len(poly) >= 4:
                    pts = np.array([[p.x, p.y] for p in poly[:4]], dtype=np.float32)
                else:
                    x, y, w, h = d.rect
                    pts = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.float32)
                results.append((text, pts))
            if results:
                return results

        try:
            ok, decoded_info, points, _ = self.detector.detectAndDecodeMulti(bgr)
            if ok and points is not None:
                for text, pts in zip(decoded_info, points):
                    if text:
                        results.append((text.strip(), np.array(pts, dtype=np.float32)))
        except Exception:
            pass
        return results

    def estimate_normal_from_qr_corners(self, pts):
        """
        Estimate QR plane normal from corner depth.
        Return normal in camera_link frame. Fallback: QR faces camera, normal roughly (-1, 0, 0).
        """
        arr = np.array(pts, dtype=np.float32)
        if arr.shape[0] < 4:
            return (-1.0, 0.0, 0.0)

        points_3d = []
        for u, v in arr[:4]:
            xyz = self.get_xyz(float(u), float(v))
            if xyz is None:
                continue
            points_3d.append(np.array(xyz[:3], dtype=np.float64))

        if len(points_3d) < 3:
            return (-1.0, 0.0, 0.0)

        p0 = points_3d[0]
        p1 = points_3d[1]
        p3 = points_3d[3] if len(points_3d) >= 4 else points_3d[2]
        v1 = p1 - p0
        v2 = p3 - p0
        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)
        if norm < 1e-6:
            return (-1.0, 0.0, 0.0)
        n = n / norm

        # QR normal should face camera. In camera_link, camera is roughly at origin and QR is in +x.
        # Vector from QR center to camera origin is approximately -center.
        center = np.mean(points_3d, axis=0)
        to_camera = -center
        if np.dot(n, to_camera) < 0:
            n = -n
        return float(n[0]), float(n[1]), float(n[2])

    def get_xyz(self, u, v):
        if self.last_depth is not None:
            depth = self.sample_depth_image(u, v)
            if depth is not None:
                x, y, z = self.deproject_depth(u, v, depth)
                return x, y, z, f'depth_image:{self.last_depth_topic}'
        if self.last_cloud is not None:
            xyz = self.sample_pointcloud(u, v)
            if xyz is not None:
                return xyz[0], xyz[1], xyz[2], f'pointcloud:{self.last_cloud_topic}'
        return None

    def sample_depth_image(self, u, v):
        depth = self.last_depth
        h, w = depth.shape[:2]
        uu = int(round(u))
        vv = int(round(v))
        if uu < 0 or uu >= w or vv < 0 or vv >= h:
            return None
        radius = self.depth_window_px
        vals = []
        for yy in range(max(0, vv - radius), min(h, vv + radius + 1)):
            for xx in range(max(0, uu - radius), min(w, uu + radius + 1)):
                d = depth[yy, xx]
                if isinstance(d, np.ndarray):
                    d = d[0]
                d = float(d)
                if depth.dtype == np.uint16:
                    if d <= 0 or d >= 65535:
                        continue
                    d = d / 1000.0
                if not math.isfinite(d):
                    continue
                if self.min_depth_m <= d <= self.max_depth_m:
                    vals.append(d)
        if not vals:
            return None
        return float(np.median(vals))

    def deproject_depth(self, u, v, z):
        k = self.camera_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]

        x_opt = (u - cx) * z / fx
        y_opt = (v - cy) * z / fy
        z_opt = z

        # optical -> camera_link: x forward, y left, z up
        x_link = z_opt
        y_link = -x_opt
        z_link = -y_opt
        return x_link, y_link, z_link

    def sample_pointcloud(self, u, v):
        cloud = self.last_cloud
        uu = int(round(u))
        vv = int(round(v))
        if uu < 0 or vv < 0 or uu >= cloud.width or vv >= cloud.height:
            return None
        names = {f.name: f.offset for f in cloud.fields}
        if not {'x', 'y', 'z'}.issubset(names):
            return None
        ox, oy, oz = names['x'], names['y'], names['z']
        vals = []
        radius = self.depth_window_px
        for yy in range(max(0, vv - radius), min(cloud.height, vv + radius + 1)):
            for xx in range(max(0, uu - radius), min(cloud.width, uu + radius + 1)):
                base = yy * cloud.row_step + xx * cloud.point_step
                try:
                    x = struct.unpack_from('f', cloud.data, base + ox)[0]
                    y = struct.unpack_from('f', cloud.data, base + oy)[0]
                    z = struct.unpack_from('f', cloud.data, base + oz)[0]
                except Exception:
                    continue
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                dist = math.sqrt(x * x + y * y + z * z)
                if self.min_depth_m <= dist <= self.max_depth_m:
                    vals.append((x, y, z))
        if not vals:
            return None
        med = np.median(np.array(vals, dtype=np.float32), axis=0)
        return float(med[0]), float(med[1]), float(med[2])

    def center_of_pts(self, pts):
        arr = np.array(pts, dtype=np.float32)
        return float(np.mean(arr[:, 0])), float(np.mean(arr[:, 1]))

    def draw_qr(self, img, text, pts, color):
        pts_i = np.array(pts, dtype=np.int32)
        cv2.polylines(img, [pts_i], True, color, 2)
        u, v = self.center_of_pts(pts)
        cv2.circle(img, (int(u), int(v)), 4, color, -1)
        cv2.putText(img, text, (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def throttled_warn(self, text):
        self.warn_count += 1
        if self.warn_count % 30 == 0:
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = VisionQRDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
