#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from pupil_apriltags import Detector


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


class D435AprilTagPupilTest(Node):
    def __init__(self, args):
        super().__init__("test_d435_apriltag_pupil")

        self.args = args
        self.bridge = CvBridge()

        self.image_count = 0
        self.info_count = 0

        self.K: Optional[np.ndarray] = None
        self.camera_params = None
        self.camera_frame = ""

        self.detector = Detector(
            families=args.family,
            nthreads=1,
            quad_decimate=args.quad_decimate,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

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

        self.get_logger().info("test_d435_apriltag_pupil started")
        self.get_logger().info(f"mode={args.mode}")
        self.get_logger().info(f"image_topic={args.image_topic}")
        self.get_logger().info(f"camera_info_topic={args.camera_info_topic}")
        self.get_logger().info(f"family={args.family}, tag_size_m={args.tag_size_m}")

    def on_camera_info(self, msg: CameraInfo):
        self.info_count += 1
        self.camera_frame = msg.header.frame_id

        k = msg.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        self.K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        self.camera_params = (fx, fy, cx, cy)

        if self.info_count == 1:
            self.get_logger().info(
                f"camera_info OK: frame={msg.header.frame_id}, "
                f"width={msg.width}, height={msg.height}, "
                f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )

    def draw_debug(self, frame, detections):
        for det in detections:
            corners = np.asarray(det.corners, dtype=np.int32)

            cv2.polylines(frame, [corners], True, (0, 255, 255), 2)

            c = np.asarray(det.center, dtype=np.int32)
            label = f"id={det.tag_id}"
            cv2.putText(
                frame,
                label,
                (int(c[0]) + 5, int(c[1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

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

        if self.camera_params is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.args.mode == "detect":
            detections = self.detector.detect(gray, estimate_tag_pose=False)
        else:
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=float(self.args.tag_size_m),
            )

        if self.image_count == 1 or self.image_count % 10 == 0:
            self.get_logger().info(
                f"detect OK: frame_count={self.image_count}, tags={len(detections)}"
            )

        frame = self.draw_debug(frame, detections)
        self.publish_debug(frame, msg.header)

        if self.args.mode == "detect":
            return

        for det in detections:
            R_opt_tag = np.asarray(det.pose_R, dtype=np.float64)
            t_opt = np.asarray(det.pose_t, dtype=np.float64).reshape(3)

            t_base, R_base_tag = optical_to_base_pose(t_opt, R_opt_tag)

            qx, qy, qz, qw = rotation_matrix_to_quaternion(R_base_tag)
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            self.get_logger().info(
                f"POSE tag_id={det.tag_id} | "
                f"t_opt=({t_opt[0]:.3f},{t_opt[1]:.3f},{t_opt[2]:.3f}) | "
                f"base=({t_base[0]:.3f},{t_base[1]:.3f},{t_base[2]:.3f}) | "
                f"quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}) | "
                f"yaw_deg={math.degrees(yaw):.2f} | "
                f"pose_err={getattr(det, 'pose_err', 0.0)}"
            )


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mode",
        choices=["detect", "pose"],
        default="detect",
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
        default="/vision/test_apriltag_pupil_debug_image",
    )

    parser.add_argument(
        "--family",
        default="tag36h11",
    )

    parser.add_argument(
        "--tag-size-m",
        type=float,
        default=0.10,
    )

    parser.add_argument(
        "--quad-decimate",
        type=float,
        default=1.0,
    )

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=None)
    node = D435AprilTagPupilTest(args)

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
