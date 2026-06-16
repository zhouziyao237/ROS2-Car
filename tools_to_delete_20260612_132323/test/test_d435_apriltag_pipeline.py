#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def rotation_matrix_to_quaternion(R: np.ndarray):
    R = np.asarray(R, dtype=np.float64)
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])

    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 1e-12)) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 1e-12)) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 1e-12)) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0

    return qx / norm, qy / norm, qz / norm, qw / norm


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def optical_to_base_pose(t_opt: np.ndarray, R_opt_tag: np.ndarray):
    """
    D435 optical frame:
      x right
      y down
      z forward

    base-like frame:
      x forward
      y left
      z up
    """
    C = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ], dtype=np.float64)

    t_base = C @ t_opt.reshape(3)
    R_base_tag = C @ R_opt_tag

    return t_base, R_base_tag


class D435AprilTagPipelineTest(Node):
    def __init__(self, args):
        super().__init__("test_d435_apriltag_pipeline")

        self.args = args
        self.bridge = CvBridge()

        self.image_count = 0
        self.info_count = 0

        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.camera_frame = ""

        self.dictionary = self.make_dictionary(args.tag_family)
        self.params = cv2.aruco.DetectorParameters()

        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.on_camera_info,
            10,
        )

        self.create_subscription(
            Image,
            args.image_topic,
            self.on_image,
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            args.debug_topic,
            10,
        )

        self.get_logger().info("test_d435_apriltag_pipeline started")
        self.get_logger().info(f"mode={args.mode}")
        self.get_logger().info(f"image_topic={args.image_topic}")
        self.get_logger().info(f"camera_info_topic={args.camera_info_topic}")
        self.get_logger().info(f"tag_family={args.tag_family}, tag_size_m={args.tag_size_m}")
        self.get_logger().info(f"cv2 version={cv2.__version__}")
        self.get_logger().info(f"has cv2.aruco={hasattr(cv2, 'aruco')}")

    def make_dictionary(self, family: str):
        table = {
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
        }

        if family not in table:
            raise RuntimeError(f"Unsupported tag family: {family}")

        return cv2.aruco.getPredefinedDictionary(table[family])

    def on_camera_info(self, msg: CameraInfo):
        self.info_count += 1
        self.camera_frame = msg.header.frame_id

        k = msg.k
        self.K = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]],
        ], dtype=np.float64)

        if msg.d:
            self.dist = np.array(msg.d, dtype=np.float64)
        else:
            self.dist = np.zeros((5,), dtype=np.float64)

        if self.info_count == 1:
            self.get_logger().info(
                f"camera_info OK: frame={msg.header.frame_id}, "
                f"width={msg.width}, height={msg.height}, "
                f"fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
            )

    def detect_markers(self, gray):
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.params,
        )
        return corners, ids, rejected

    def estimate_pose(self, pts_2d: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self.K is None:
            return None

        s = float(self.args.tag_size_m)

        obj_pts = np.array([
            [-s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0, -s / 2.0, 0.0],
            [-s / 2.0, -s / 2.0, 0.0],
        ], dtype=np.float64)

        img_pts = np.asarray(pts_2d, dtype=np.float64).reshape(4, 2)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            self.K,
            self.dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok:
            return None

        return rvec.reshape(3, 1), tvec.reshape(3, 1)

    def draw_debug(self, frame, corners, ids):
        if ids is None:
            return frame

        try:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        except Exception:
            pass

        return frame

    def publish_debug(self, frame, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = header
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"debug publish failed: {e}")

    def on_image(self, msg: Image):
        self.image_count += 1

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.image_count == 1 or self.image_count % 30 == 0:
            self.get_logger().info(
                f"image OK: count={self.image_count}, shape={frame.shape}, "
                f"encoding={msg.encoding}, frame={msg.header.frame_id}"
            )

        if self.args.mode == "image":
            if self.image_count % 30 == 0:
                self.publish_debug(frame, msg.header)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detect_markers(gray)

        n = 0 if ids is None else len(ids)

        if self.image_count == 1 or self.image_count % 10 == 0:
            self.get_logger().info(f"detect OK: frame_count={self.image_count}, tags={n}")

        frame = self.draw_debug(frame, corners, ids)
        self.publish_debug(frame, msg.header)

        if self.args.mode == "detect":
            return

        if ids is None or len(ids) == 0:
            return

        for i, tag_id_np in enumerate(ids.flatten()):
            tag_id = int(tag_id_np)
            pts = corners[i].reshape(4, 2)

            pose = self.estimate_pose(pts)
            if pose is None:
                self.get_logger().warn(f"solvePnP failed for tag_id={tag_id}")
                continue

            rvec, tvec = pose
            R_opt_tag, _ = cv2.Rodrigues(rvec)

            t_opt = tvec.reshape(3)
            t_base, R_base_tag = optical_to_base_pose(t_opt, R_opt_tag)

            qx, qy, qz, qw = rotation_matrix_to_quaternion(R_base_tag)
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            self.get_logger().info(
                f"POSE tag_id={tag_id} | "
                f"t_opt=({t_opt[0]:.3f},{t_opt[1]:.3f},{t_opt[2]:.3f}) | "
                f"base=({t_base[0]:.3f},{t_base[1]:.3f},{t_base[2]:.3f}) | "
                f"rvec=({float(rvec[0]):.3f},{float(rvec[1]):.3f},{float(rvec[2]):.3f}) | "
                f"quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}) | "
                f"yaw_deg={math.degrees(yaw):.2f}"
            )


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mode",
        choices=["image", "detect", "pose"],
        default="image",
        help="image=只测图像; detect=测AprilTag检测; pose=测AprilTag 6D pose",
    )

    parser.add_argument(
        "--image-topic",
        default="/camera/camera/color/image_raw",
    )

    parser.add_argument(
        "--camera-info-topic",
        default="/camera/camera/color/camera_info",
    )

    parser.add_argument(
        "--debug-topic",
        default="/vision/test_apriltag_debug_image",
    )

    parser.add_argument(
        "--tag-family",
        default="DICT_APRILTAG_36h11",
    )

    parser.add_argument(
        "--tag-size-m",
        type=float,
        default=0.10,
    )

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=None)
    node = D435AprilTagPipelineTest(args)

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
