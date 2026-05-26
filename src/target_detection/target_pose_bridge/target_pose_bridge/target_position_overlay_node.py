#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from std_msgs.msg import String

from yolo_msg.msg import YoloDetectionArray


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_vec(v):
    n = np.linalg.norm(v)
    if n < 1e-6:
        return None
    return v / n


def quat_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class TargetPositionOverlayNode(Node):
    def __init__(self):
        super().__init__('target_position_overlay_node')

        # =========================
        # Topics
        # =========================
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('detections_topic', '/target_matcher/matched_detection')

        self.declare_parameter('target_pose_topic', '/target_pose_base')
        self.declare_parameter('target_pose_3d_topic', '/target_pose_3d')
        self.declare_parameter('overlay_image_topic', '/target_position_overlay/image')
        self.declare_parameter('debug_topic', '/target_position_overlay/debug')

        self.declare_parameter('target_frame', 'oak_camera_link')

        # =========================
        # Depth sampling
        # =========================
        self.declare_parameter('depth_window_px', 5)
        self.declare_parameter('min_depth_mm', 50.0)
        self.declare_parameter('max_depth_mm', 5000.0)

        self.declare_parameter('use_grid_points', True)
        self.declare_parameter('grid_rows', 3)
        self.declare_parameter('grid_cols', 3)

        # bbox 四角向内部缩进，避开边缘深度空洞
        self.declare_parameter('corner_inset_ratio', 0.18)

        # 中心深度和多点深度差距太大只标记，不直接丢弃
        self.declare_parameter('depth_consistency_max_diff_mm', 150.0)

        # 有效深度点数量少于这个值，不发布 /target_pose_base
        self.declare_parameter('min_valid_depth_points', 3)

        # 关键：RGB 1920×1080，Depth 低分辨率时，自动缩放坐标
        self.declare_parameter('scale_rgb_to_depth', True)

        # 是否在 overlay 上显示 depth 尺寸信息
        self.declare_parameter('draw_size_info', True)

        # =========================
        # Plane angle estimation
        # =========================
        self.declare_parameter('enable_plane_angle_estimation', True)
        self.declare_parameter('min_valid_corner_points', 3)

        # =========================
        # Overlay drawing
        # =========================
        self.declare_parameter('draw_overlay', True)
        self.declare_parameter('draw_sample_points', True)
        self.declare_parameter('draw_camera_center', True)
        self.declare_parameter('draw_image_center_line', True)
        self.declare_parameter('draw_alignment_target', True)
        self.declare_parameter('draw_alignment_lines', True)

        # 坐标定义：
        # x = 左右偏移
        # y = 升降偏移
        # z = 深度距离
        #
        # 红色 pickup target 对齐点
        self.declare_parameter('align_target_x_mm', 0.0)
        self.declare_parameter('align_target_y_mm', 0.0)

        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('verbose', True)

        self.bridge = CvBridge()

        self.last_image_msg = None
        self.last_depth_msg = None
        self.last_camera_info = None
        self.last_detections = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.last_pub_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(
            PointStamped,
            self.get_parameter('target_pose_topic').value,
            10
        )

        self.pose3d_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('target_pose_3d_topic').value,
            10
        )

        self.overlay_pub = self.create_publisher(
            Image,
            self.get_parameter('overlay_image_topic').value,
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            self.get_parameter('debug_topic').value,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_cb,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_cb,
            10
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_cb,
            10
        )

        self.create_subscription(
            YoloDetectionArray,
            self.get_parameter('detections_topic').value,
            self.detections_cb,
            10
        )

        self.create_timer(0.02, self.loop)

        self.get_logger().info('target_position_overlay_node started')
        self.get_logger().info('RGB is main image; depth sampling uses RGB->depth coordinate scaling')
        self.get_logger().info(f'image_topic={self.get_parameter("image_topic").value}')
        self.get_logger().info(f'depth_topic={self.get_parameter("depth_topic").value}')
        self.get_logger().info(f'camera_info_topic={self.get_parameter("camera_info_topic").value}')
        self.get_logger().info(f'detections_topic={self.get_parameter("detections_topic").value}')
        self.get_logger().info(f'target_frame={self.get_parameter("target_frame").value}')

    # =========================
    # Logging
    # =========================
    def log_debug(self, text):
        if bool(self.get_parameter('verbose').value):
            msg = String()
            msg.data = str(text)
            self.debug_pub.publish(msg)
            self.get_logger().info(str(text))

    # =========================
    # Callbacks
    # =========================
    def image_cb(self, msg):
        self.last_image_msg = msg

    def depth_cb(self, msg):
        self.last_depth_msg = msg

    def camera_info_cb(self, msg):
        self.last_camera_info = msg

        if len(msg.k) >= 9:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def detections_cb(self, msg):
        # 关键：收到空 detection 时清空旧目标
        if len(msg.detections) == 0:
            self.last_detections = None
            return

        self.last_detections = msg

    # =========================
    # Image conversion
    # =========================
    def convert_depth_image(self):
        if self.last_depth_msg is None:
            return None

        try:
            depth_img = self.bridge.imgmsg_to_cv2(
                self.last_depth_msg,
                desired_encoding='passthrough'
            )
            return depth_img
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')
            return None

    def rgb_to_depth_uv(self, u_rgb, v_rgb, depth_img):
        """
        detection 的 u/v 是 RGB 主图坐标。
        如果 RGB 是 1920×1080，而 depth 是 640×400，
        这里会自动把 RGB 坐标映射到 depth 坐标。
        """
        if depth_img is None:
            return None, None

        depth_h, depth_w = depth_img.shape[:2]

        if not bool(self.get_parameter('scale_rgb_to_depth').value):
            return float(u_rgb), float(v_rgb)

        if self.last_image_msg is None:
            return float(u_rgb), float(v_rgb)

        rgb_w = float(self.last_image_msg.width)
        rgb_h = float(self.last_image_msg.height)

        if rgb_w <= 0.0 or rgb_h <= 0.0:
            return float(u_rgb), float(v_rgb)

        u_depth = float(u_rgb) * float(depth_w) / rgb_w
        v_depth = float(v_rgb) * float(depth_h) / rgb_h

        return u_depth, v_depth

    def get_depth_mm_at(self, depth_img, u_rgb, v_rgb, window_px=None):
        if depth_img is None:
            return None

        depth_h, depth_w = depth_img.shape[:2]

        if window_px is None:
            window_px = int(self.get_parameter('depth_window_px').value)

        window_px = max(1, int(window_px))

        u_depth, v_depth = self.rgb_to_depth_uv(u_rgb, v_rgb, depth_img)

        if u_depth is None or v_depth is None:
            return None

        u = int(round(u_depth))
        v = int(round(v_depth))

        if u < 0 or u >= depth_w or v < 0 or v >= depth_h:
            return None

        x1 = max(0, u - window_px)
        x2 = min(depth_w, u + window_px + 1)
        y1 = max(0, v - window_px)
        y2 = min(depth_h, v + window_px + 1)

        roi = depth_img[y1:y2, x1:x2]

        if roi.size == 0:
            return None

        values = roi.astype(np.float32).reshape(-1)

        # OAK depth 通常是 uint16 mm；如果是 float，一般是 meter
        if depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
            values = values * 1000.0

        values = values[np.isfinite(values)]

        min_d = float(self.get_parameter('min_depth_mm').value)
        max_d = float(self.get_parameter('max_depth_mm').value)

        values = values[(values >= min_d) & (values <= max_d)]

        if values.size == 0:
            return None

        return float(np.median(values))

    # =========================
    # Sampling points
    # =========================
    def get_bbox_points(self, det):
        b = det.box

        x1 = float(b.x1)
        y1 = float(b.y1)
        x2 = float(b.x2)
        y2 = float(b.y2)

        cu = float(b.center_u)
        cv = float(b.center_v)

        w = max(1.0, x2 - x1)
        h = max(1.0, y2 - y1)

        inset = float(self.get_parameter('corner_inset_ratio').value)
        inset = clamp(inset, 0.0, 0.45)

        ix = w * inset
        iy = h * inset

        pts = []

        # 中心
        pts.append(('center', cu, cv))

        # 四角向内缩
        pts.append(('lt', x1 + ix, y1 + iy))
        pts.append(('rt', x2 - ix, y1 + iy))
        pts.append(('rb', x2 - ix, y2 - iy))
        pts.append(('lb', x1 + ix, y2 - iy))

        # 四边中点
        pts.append(('top', cu, y1 + iy))
        pts.append(('bottom', cu, y2 - iy))
        pts.append(('left', x1 + ix, cv))
        pts.append(('right', x2 - ix, cv))

        # bbox 内部网格
        if bool(self.get_parameter('use_grid_points').value):
            rows = max(1, int(self.get_parameter('grid_rows').value))
            cols = max(1, int(self.get_parameter('grid_cols').value))

            for r in range(rows):
                for c in range(cols):
                    if rows == 1:
                        vv = cv
                    else:
                        vv = y1 + iy + (y2 - y1 - 2 * iy) * r / (rows - 1)

                    if cols == 1:
                        uu = cu
                    else:
                        uu = x1 + ix + (x2 - x1 - 2 * ix) * c / (cols - 1)

                    pts.append((f'g{r}{c}', uu, vv))

        return pts

    def sample_depths(self, depth_img, det):
        pts = self.get_bbox_points(det)

        samples = []

        for name, u, v in pts:
            d = self.get_depth_mm_at(depth_img, u, v)

            if d is not None:
                samples.append({
                    'name': name,
                    'u_rgb': float(u),
                    'v_rgb': float(v),
                    'depth_mm': float(d),
                })

        return samples

    def robust_depth_from_samples(self, samples):
        if not samples:
            return None, None

        center_depth = None

        for s in samples:
            if s['name'] == 'center':
                center_depth = s['depth_mm']
                break

        values = np.array([s['depth_mm'] for s in samples], dtype=np.float32)

        if values.size == 0:
            return None, None

        median_depth = float(np.median(values))

        # MAD 去掉离群点
        abs_dev = np.abs(values - median_depth)
        mad = float(np.median(abs_dev))

        if mad < 1.0:
            filtered = values
        else:
            filtered = values[abs_dev <= max(80.0, 3.0 * mad)]

        if filtered.size == 0:
            filtered = values

        robust_depth = float(np.median(filtered))

        consistency_ok = True

        if center_depth is not None:
            max_diff = float(self.get_parameter('depth_consistency_max_diff_mm').value)
            if abs(center_depth - robust_depth) > max_diff:
                consistency_ok = False

        info = {
            'center_depth_mm': center_depth,
            'median_depth_mm': median_depth,
            'robust_depth_mm': robust_depth,
            'valid_count': len(samples),
            'filtered_count': int(filtered.size),
            'consistency_ok': consistency_ok,
        }

        return robust_depth, info

    # =========================
    # Projection
    # =========================
    def pixel_depth_to_xyz(self, u_rgb, v_rgb, depth_mm):
        """
        输出坐标定义：
        x = 左右偏移
        y = 升降偏移
        z = 深度距离

        注意：这里使用 RGB 相机内参，因为 u/v 来自 RGB 主画面。
        """
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        z_m = float(depth_mm) / 1000.0
        x_m = (float(u_rgb) - self.cx) * z_m / self.fx
        y_m = (float(v_rgb) - self.cy) * z_m / self.fy

        return np.array([x_m, y_m, z_m], dtype=np.float32)

    def make_pose_msg(self, u_rgb, v_rgb, depth_mm):
        p = self.pixel_depth_to_xyz(u_rgb, v_rgb, depth_mm)

        if p is None:
            return None

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('target_frame').value

        msg.point.x = float(p[0])
        msg.point.y = float(p[1])
        msg.point.z = float(p[2])

        return msg

    def project_xyz_to_pixel(self, x_mm, y_mm, z_mm):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        z_m = float(z_mm) / 1000.0
        if z_m <= 0.001:
            return None

        x_m = float(x_mm) / 1000.0
        y_m = float(y_mm) / 1000.0

        u = self.cx + self.fx * x_m / z_m
        v = self.cy + self.fy * y_m / z_m

        return int(round(u)), int(round(v))

    def project_alignment_target(self, depth_mm):
        x_mm = float(self.get_parameter('align_target_x_mm').value)
        y_mm = float(self.get_parameter('align_target_y_mm').value)
        z_mm = float(depth_mm)

        return self.project_xyz_to_pixel(x_mm, y_mm, z_mm)

    # =========================
    # Plane angle estimation
    # =========================
    def estimate_plane_angles(self, depth_img, det):
        """
        用 bbox 内缩四角点近似目标平面姿态。
        注意：这不是 QR 真四角点，但足够先看 yaw / pitch 趋势。
        """
        b = det.box

        x1 = float(b.x1)
        y1 = float(b.y1)
        x2 = float(b.x2)
        y2 = float(b.y2)

        w = max(1.0, x2 - x1)
        h = max(1.0, y2 - y1)

        inset = float(self.get_parameter('corner_inset_ratio').value)
        inset = clamp(inset, 0.0, 0.45)

        ix = w * inset
        iy = h * inset

        corner_uv = [
            ('lt', x1 + ix, y1 + iy),
            ('rt', x2 - ix, y1 + iy),
            ('rb', x2 - ix, y2 - iy),
            ('lb', x1 + ix, y2 - iy),
        ]

        pts_3d = {}

        for name, u, v in corner_uv:
            d = self.get_depth_mm_at(depth_img, u, v)

            if d is None:
                continue

            p = self.pixel_depth_to_xyz(u, v, d)

            if p is not None:
                pts_3d[name] = p

        min_valid = int(self.get_parameter('min_valid_corner_points').value)

        if len(pts_3d) < min_valid:
            return None

        if 'lt' in pts_3d and 'rt' in pts_3d and 'lb' in pts_3d:
            p0 = pts_3d['lt']
            p1 = pts_3d['rt']
            p3 = pts_3d['lb']
        elif 'rt' in pts_3d and 'rb' in pts_3d and 'lt' in pts_3d:
            p0 = pts_3d['rt']
            p1 = pts_3d['rb']
            p3 = pts_3d['lt']
        else:
            keys = list(pts_3d.keys())
            p0 = pts_3d[keys[0]]
            p1 = pts_3d[keys[1]]
            p3 = pts_3d[keys[2]]

        v1 = p1 - p0
        v2 = p3 - p0

        normal = np.cross(v1, v2)
        normal = normalize_vec(normal)

        if normal is None:
            return None

        # 坐标定义：
        # x = 左右
        # y = 上下
        # z = 深度
        nx, ny, nz = normal

        yaw = math.atan2(nx, nz)
        pitch = math.atan2(-ny, math.sqrt(nx * nx + nz * nz))
        roll = 0.0

        return {
            'normal': normal,
            'yaw_deg': math.degrees(yaw),
            'pitch_deg': math.degrees(pitch),
            'roll_deg': math.degrees(roll),
            'corner_count': len(pts_3d),
        }

    def publish_pose3d(self, point_msg, angle_info):
        pose_msg = PoseStamped()
        pose_msg.header = point_msg.header

        pose_msg.pose.position.x = point_msg.point.x
        pose_msg.pose.position.y = point_msg.point.y
        pose_msg.pose.position.z = point_msg.point.z

        if angle_info is None:
            pose_msg.pose.orientation.w = 1.0
        else:
            yaw = math.radians(angle_info['yaw_deg'])
            pitch = math.radians(angle_info['pitch_deg'])
            roll = math.radians(angle_info['roll_deg'])
            pose_msg.pose.orientation = quat_from_euler(roll, pitch, yaw)

        self.pose3d_pub.publish(pose_msg)

    # =========================
    # Drawing helpers
    # =========================
    def draw_cross(self, img, u, v, color, label):
        h, w = img.shape[:2]

        u = int(round(u))
        v = int(round(v))

        if u < 0 or u >= w or v < 0 or v >= h:
            return

        cv2.drawMarker(
            img,
            (u, v),
            color,
            markerType=cv2.MARKER_CROSS,
            markerSize=22,
            thickness=2
        )

        cv2.circle(img, (u, v), 5, color, 2)

        cv2.putText(
            img,
            label,
            (u + 7, max(20, v - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1
        )

    def draw_overlay(self, img, det, samples, depth_info, angle_info, pose_msg, depth_img):
        b = det.box

        x1 = int(b.x1)
        y1 = int(b.y1)
        x2 = int(b.x2)
        y2 = int(b.y2)
        cu = int(b.center_u)
        cvp = int(b.center_v)

        h, w = img.shape[:2]

        # QR bbox
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 绿色十字：目标中心
        self.draw_cross(img, cu, cvp, (0, 255, 0), 'target center')

        # 蓝色十字：相机中心 / 图像中心
        if self.cx is not None and self.cy is not None:
            center_u = int(round(self.cx))
            center_v = int(round(self.cy))
        else:
            center_u = int(w / 2)
            center_v = int(h / 2)

        if bool(self.get_parameter('draw_camera_center').value):
            self.draw_cross(img, center_u, center_v, (255, 0, 0), 'image center')

        # 目标中心到图像中心的显示线
        if bool(self.get_parameter('draw_image_center_line').value):
            cv2.line(img, (cu, cvp), (center_u, center_v), (0, 0, 255), 2)
            cv2.line(img, (cu, cvp), (center_u, cvp), (0, 255, 255), 2)
            cv2.line(img, (center_u, cvp), (center_u, center_v), (255, 0, 255), 2)

            du = cu - center_u
            dv = cvp - center_v

            cv2.putText(
                img,
                f'target-center error: dx={du:+d}px dy={dv:+d}px',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )

        # 深度采样点
        if bool(self.get_parameter('draw_sample_points').value):
            for s in samples:
                name = s['name']
                u = int(round(s['u_rgb']))
                v = int(round(s['v_rgb']))

                color = (0, 255, 255)
                if name == 'center':
                    color = (255, 255, 0)

                cv2.circle(img, (u, v), 3, color, -1)

        # 红色 pickup target
        if bool(self.get_parameter('draw_alignment_target').value):
            uv = self.project_alignment_target(depth_info['robust_depth_mm'])

            if uv is not None:
                target_u, target_v = uv

                self.draw_cross(img, target_u, target_v, (0, 0, 255), 'pickup target')

                if bool(self.get_parameter('draw_alignment_lines').value):
                    # 红色斜线：QR 当前中心 -> 平台对齐点
                    cv2.line(img, (cu, cvp), (target_u, target_v), (0, 0, 255), 2)

                    # 黄色横线：x 左右误差
                    cv2.line(img, (cu, cvp), (target_u, cvp), (0, 255, 255), 2)

                    # 紫色竖线：y 升降误差
                    cv2.line(img, (target_u, cvp), (target_u, target_v), (255, 0, 255), 2)

                    du_pickup = target_u - cu
                    dv_pickup = target_v - cvp

                    cv2.putText(
                        img,
                        f'pickup error: du={du_pickup:+d}px dv={dv_pickup:+d}px',
                        (10, 58),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 255),
                        2
                    )

        # 坐标显示
        text1 = (
            f"x={pose_msg.point.x*1000.0:.1f} "
            f"y={pose_msg.point.y*1000.0:.1f} "
            f"z={pose_msg.point.z*1000.0:.1f} mm"
        )

        cv2.putText(
            img,
            text1,
            (10, img.shape[0] - 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2
        )

        c_depth = depth_info['center_depth_mm']
        c_text = 'None' if c_depth is None else f'{c_depth:.1f}'

        text2 = (
            f"depth center={c_text} robust={depth_info['robust_depth_mm']:.1f} "
            f"valid={depth_info['valid_count']}/{depth_info['filtered_count']} "
            f"ok={depth_info['consistency_ok']}"
        )

        cv2.putText(
            img,
            text2,
            (10, img.shape[0] - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1
        )

        if angle_info is not None:
            text3 = (
                f"yaw={angle_info['yaw_deg']:.1f} "
                f"pitch={angle_info['pitch_deg']:.1f} "
                f"corners={angle_info['corner_count']}"
            )
        else:
            text3 = "yaw=None pitch=None"

        cv2.putText(
            img,
            text3,
            (10, img.shape[0] - 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1
        )

        if bool(self.get_parameter('draw_size_info').value) and depth_img is not None:
            dh, dw = depth_img.shape[:2]
            text4 = f"RGB={w}x{h} Depth={dw}x{dh} scale_depth={self.get_parameter('scale_rgb_to_depth').value}"
            cv2.putText(
                img,
                text4,
                (10, img.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1
            )

    # =========================
    # Main loop
    # =========================
    def select_best_detection(self):
        if self.last_detections is None:
            return None

        if len(self.last_detections.detections) == 0:
            return None

        return max(
            self.last_detections.detections,
            key=lambda d: float(d.confidence)
        )

    def loop(self):
        if self.last_image_msg is None:
            return

        now = self.get_clock().now()
        rate = float(self.get_parameter('publish_rate_hz').value)

        if rate > 0.0:
            dt = (now - self.last_pub_time).nanoseconds * 1e-9
            if dt < 1.0 / rate:
                return

        self.last_pub_time = now

        try:
            img = self.bridge.imgmsg_to_cv2(self.last_image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        depth_img = self.convert_depth_image()
        best_det = self.select_best_detection()

        if best_det is None or depth_img is None:
            if bool(self.get_parameter('draw_overlay').value):
                self.overlay_pub.publish(
                    self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                )
            return

        samples = self.sample_depths(depth_img, best_det)

        min_valid = int(self.get_parameter('min_valid_depth_points').value)

        if len(samples) < min_valid:
            self.log_debug(f'NO_POSE not enough depth samples {len(samples)}/{min_valid}')

            if bool(self.get_parameter('draw_overlay').value):
                # 即使没有深度，也画出目标框和中心，方便调试
                b = best_det.box
                x1 = int(b.x1)
                y1 = int(b.y1)
                x2 = int(b.x2)
                y2 = int(b.y2)
                cu = int(b.center_u)
                cvp = int(b.center_v)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.draw_cross(img, cu, cvp, (0, 255, 0), 'target center')

                if self.cx is not None and self.cy is not None:
                    self.draw_cross(img, self.cx, self.cy, (255, 0, 0), 'image center')

                if depth_img is not None:
                    dh, dw = depth_img.shape[:2]
                    cv2.putText(
                        img,
                        f'NO DEPTH samples={len(samples)}/{min_valid} RGB={img.shape[1]}x{img.shape[0]} Depth={dw}x{dh}',
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 0, 255),
                        2
                    )

                self.overlay_pub.publish(
                    self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                )
            return

        depth_mm, depth_info = self.robust_depth_from_samples(samples)

        if depth_mm is None:
            self.log_debug('NO_POSE no robust depth')

            if bool(self.get_parameter('draw_overlay').value):
                self.overlay_pub.publish(
                    self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                )
            return

        cu = float(best_det.box.center_u)
        cvp = float(best_det.box.center_v)

        pose_msg = self.make_pose_msg(cu, cvp, depth_mm)

        if pose_msg is None:
            return

        angle_info = None

        if bool(self.get_parameter('enable_plane_angle_estimation').value):
            angle_info = self.estimate_plane_angles(depth_img, best_det)

        self.pose_pub.publish(pose_msg)
        self.publish_pose3d(pose_msg, angle_info)

        yaw_text = 'None' if angle_info is None else f"{angle_info['yaw_deg']:.1f}"
        pitch_text = 'None' if angle_info is None else f"{angle_info['pitch_deg']:.1f}"

        self.log_debug(
            f'target {pose_msg.header.frame_id}: '
            f'x={pose_msg.point.x * 1000.0:.1f} '
            f'y={pose_msg.point.y * 1000.0:.1f} '
            f'z={pose_msg.point.z * 1000.0:.1f} mm | '
            f'depth_center={depth_info["center_depth_mm"]} '
            f'depth_robust={depth_info["robust_depth_mm"]:.1f} '
            f'valid={depth_info["valid_count"]} '
            f'ok={depth_info["consistency_ok"]} '
            f'yaw={yaw_text} pitch={pitch_text}'
        )

        if bool(self.get_parameter('draw_overlay').value):
            self.draw_overlay(img, best_det, samples, depth_info, angle_info, pose_msg, depth_img)
            self.overlay_pub.publish(
                self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionOverlayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()