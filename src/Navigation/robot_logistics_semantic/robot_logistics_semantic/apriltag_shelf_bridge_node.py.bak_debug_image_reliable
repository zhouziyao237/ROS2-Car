#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from typing import Optional, Dict, Any

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


def parse_shelf_tag(tag_id: int) -> Dict[str, Any]:
    """
    AprilTag 编码规则：

    1. HOME 标签：
       tag_id = 0
       表示 HOME / MAP_ORIGIN / 场地语义起点。

    2. 货架标签：
       tag_id = shelf_number * 10 + row

       row = 0:
         侧面列标签，对应旧系统 column_qr。

       row > 0:
         正面层/行标签，对应旧系统 layer_qr。

    示例：
       tag_id 30 -> S03 侧面，显示码 0300
       tag_id 31 -> S03-L1 正面，显示码 0301
       tag_id 32 -> S03-L2 正面，显示码 0302
    """
    tag_id = int(tag_id)

    if tag_id == 0:
        return {
            "valid": True,
            "tag_id": 0,
            "raw": "HOME",
            "data": "HOME",
            "display_code": "HOME",

            "type": "home_tag",
            "obs_type": "home_tag",
            "qr_type": "home_tag",
            "tag_type": "home_tag",

            "face": "home",
            "shelf_id": "",
            "shelf": "",
            "shelf_number": 0,

            "col": 0,
            "column": 0,
            "row": 0,

            "layer_id": "",
            "layer": "",
        }

    shelf_number = tag_id // 10
    row = tag_id % 10

    if shelf_number <= 0:
        return {
            "valid": False,
            "reason": f"invalid shelf_number from tag_id={tag_id}",
        }

    shelf_id = f"S{shelf_number:02d}"
    display_code = f"{shelf_number:02d}{row:02d}"

    if row == 0:
        return {
            "valid": True,
            "tag_id": tag_id,
            "raw": display_code,
            "data": display_code,
            "display_code": display_code,

            "type": "column_qr",
            "obs_type": "column_qr",
            "qr_type": "column_qr",
            "tag_type": "column_tag",

            "face": "side",
            "shelf_id": shelf_id,
            "shelf": shelf_id,
            "shelf_number": shelf_number,

            "col": shelf_number,
            "column": shelf_number,
            "row": 0,

            "layer_id": "",
            "layer": "",
        }

    return {
        "valid": True,
        "tag_id": tag_id,
        "raw": display_code,
        "data": display_code,
        "display_code": display_code,

        "type": "layer_qr",
        "obs_type": "layer_qr",
        "qr_type": "layer_qr",
        "tag_type": "row_tag",

        "face": "front",
        "shelf_id": shelf_id,
        "shelf": shelf_id,
        "shelf_number": shelf_number,

        "col": shelf_number,
        "column": shelf_number,
        "row": row,

        "layer_id": f"L{row}",
        "layer": f"L{row}",
    }


class AprilTagShelfBridgeNode(Node):
    """
    AprilTag 货架语义桥接节点。

    输入：
      /camera/camera/color/image_raw
      /camera/camera/color/camera_info

    输出：
      /vision/semantic_observation

    作用：
      用 AprilTag 替代旧货架 QR。
      但是输出仍然使用旧系统兼容字段：
        layer_qr
        column_qr
        camera_xyz
        normal
        shelf_id
        layer_id

      这样 semantic_registrar_node、semantic_library.json、mission_btcpp_node
      都可以继续沿用原来的逻辑。
    """

    def __init__(self):
        super().__init__("apriltag_shelf_bridge_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("observation_topic", "/vision/semantic_observation")
        self.declare_parameter("debug_image_topic", "/vision/apriltag_debug_image")

        self.declare_parameter("tag_family", "DICT_APRILTAG_36h11")
        self.declare_parameter("tag_size_m", 0.10)

        self.declare_parameter("min_area_px", 250.0)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("frame_id_fallback", "camera_color_optical_frame")
        self.declare_parameter("source_name", "apriltag_shelf_bridge")
        self.declare_parameter("debug_log", True)

        # 如果 RViz 里货架朝向反了 180 度，优先试着把这个改成 false。
        self.declare_parameter("normal_points_to_camera", True)

        self.bridge = CvBridge()

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_frame = str(self.get_parameter("frame_id_fallback").value)

        self.dictionary = self.create_dictionary()
        self.detector = self.create_detector()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info,
            10,
        )

        self.obs_pub = self.create_publisher(
            String,
            str(self.get_parameter("observation_topic").value),
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            sensor_qos,
        )

        self.get_logger().info(
            "apriltag_shelf_bridge_node started: "
            f"image={self.get_parameter('image_topic').value}, "
            f"camera_info={self.get_parameter('camera_info_topic').value}, "
            f"observation={self.get_parameter('observation_topic').value}, "
            f"tag_size_m={self.get_parameter('tag_size_m').value}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def create_dictionary(self):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "cv2.aruco not found. Install python3-opencv or opencv-contrib-python."
            )

        aruco = cv2.aruco
        family_name = str(self.get_parameter("tag_family").value)

        family_map = {
            "DICT_APRILTAG_16h5": getattr(aruco, "DICT_APRILTAG_16h5", None),
            "DICT_APRILTAG_25h9": getattr(aruco, "DICT_APRILTAG_25h9", None),
            "DICT_APRILTAG_36h10": getattr(aruco, "DICT_APRILTAG_36h10", None),
            "DICT_APRILTAG_36h11": getattr(aruco, "DICT_APRILTAG_36h11", None),
        }

        if family_name not in family_map or family_map[family_name] is None:
            raise RuntimeError(f"Unsupported OpenCV AprilTag family: {family_name}")

        if hasattr(aruco, "getPredefinedDictionary"):
            return aruco.getPredefinedDictionary(family_map[family_name])

        return aruco.Dictionary_get(family_map[family_name])

    def create_detector(self):
        aruco = cv2.aruco

        if hasattr(aruco, "ArucoDetector"):
            params = aruco.DetectorParameters()
            try:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            except Exception:
                pass
            return aruco.ArucoDetector(self.dictionary, params)

        return None

    def on_camera_info(self, msg: CameraInfo):
        self.camera_frame = msg.header.frame_id or str(self.get_parameter("frame_id_fallback").value)
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def detect_markers(self, gray):
        aruco = cv2.aruco

        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
            return corners, ids, rejected

        params = aruco.DetectorParameters_create()
        corners, ids, rejected = aruco.detectMarkers(gray, self.dictionary, parameters=params)
        return corners, ids, rejected

    def marker_area(self, corners) -> float:
        pts = np.asarray(corners).reshape(4, 2).astype(np.float32)
        return float(cv2.contourArea(pts))

    def estimate_pose(self, corners):
        if self.camera_matrix is None:
            return None, None

        tag_size = float(self.get_parameter("tag_size_m").value)
        half = tag_size / 2.0

        objp = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        imgp = np.asarray(corners, dtype=np.float64).reshape(4, 2)

        flags = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE

        ok, rvec, tvec = cv2.solvePnP(
            objp,
            imgp,
            self.camera_matrix,
            self.dist_coeffs,
            flags=flags,
        )

        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                objp,
                imgp,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        if not ok:
            return None, None

        return rvec, tvec

    def make_normal(self, rvec):
        rmat, _ = cv2.Rodrigues(rvec)
        normal = rmat[:, 2].reshape(3)

        if bool(self.get_parameter("normal_points_to_camera").value):
            # OpenCV optical frame:
            # +Z 是相机向前看。
            # 可见标签平面的法向通常应大致朝向相机，即 -Z。
            if normal[2] > 0:
                normal = -normal

        return normal

    def make_observation(self, msg: Image, tag_id: int, corners, rvec, tvec):
        meta = parse_shelf_tag(tag_id)

        if not meta.get("valid", False):
            return None

        tv = np.asarray(tvec).reshape(3)
        rv = np.asarray(rvec).reshape(3)
        normal = self.make_normal(rvec)

        pts = np.asarray(corners).reshape(4, 2)
        center = pts.mean(axis=0)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.now_sec()

        obs = {
            **meta,

            "camera_frame": self.camera_frame,
            "frame_id": self.camera_frame,

            # OpenCV optical camera coordinates:
            # x right, y down, z forward.
            "camera_xyz": [float(tv[0]), float(tv[1]), float(tv[2])],
            "normal": [float(normal[0]), float(normal[1]), float(normal[2])],
            "rvec": [float(rv[0]), float(rv[1]), float(rv[2])],

            "image_center": [float(center[0]), float(center[1])],
            "corners": pts.astype(float).tolist(),

            "confidence": 0.95,
            "stamp": float(stamp_sec),
            "source": str(self.get_parameter("source_name").value),
        }

        return obs

    def publish_observation(self, obs: Dict[str, Any]):
        out = String()
        out.data = json.dumps(obs, ensure_ascii=False)
        self.obs_pub.publish(out)

        if bool(self.get_parameter("debug_log").value):
            xyz = obs.get("camera_xyz", [0, 0, 0])
            normal = obs.get("normal", [0, 0, 0])
            self.get_logger().info(
                f"AprilTag obs: "
                f"tag_id={obs.get('tag_id')}, raw={obs.get('raw')}, "
                f"type={obs.get('type')}, face={obs.get('face')}, "
                f"shelf={obs.get('shelf_id')}, layer={obs.get('layer_id')}, "
                f"camera_xyz=({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f}), "
                f"normal=({normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f})"
            )

    def on_image(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {repr(e)}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        try:
            corners, ids, _ = self.detect_markers(gray)
        except Exception as e:
            self.get_logger().warn(f"AprilTag detect failed: {repr(e)}")
            return

        debug = frame.copy()

        if ids is not None and len(ids) > 0:
            try:
                cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            except Exception:
                pass

            for i, arr in enumerate(ids):
                tag_id = int(arr[0])
                cs = corners[i]

                area = self.marker_area(cs)
                if area < float(self.get_parameter("min_area_px").value):
                    continue

                rvec, tvec = self.estimate_pose(cs)
                if rvec is None or tvec is None:
                    continue

                obs = self.make_observation(msg, tag_id, cs, rvec, tvec)
                if obs is not None:
                    self.publish_observation(obs)

                try:
                    cv2.drawFrameAxes(
                        debug,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        float(self.get_parameter("tag_size_m").value) * 0.5,
                    )
                except Exception:
                    pass

        if bool(self.get_parameter("publish_debug_image").value):
            try:
                out = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                out.header = msg.header
                self.debug_pub.publish(out)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagShelfBridgeNode()

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


if __name__ == "__main__":
    main()
