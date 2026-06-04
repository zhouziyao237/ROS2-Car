#!/usr/bin/env python3
import argparse
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ChessboardCaptureNode(Node):
    def __init__(self, args):
        super().__init__('oak_chessboard_capture')
        self.args = args
        self.bridge = CvBridge()
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.pattern_size = (args.cols, args.rows)
        self.saved_count = 0
        self.last_save_time = 0.0
        self.last_center = None
        self.last_area = None

        self.sub = self.create_subscription(Image, args.image_topic, self.image_callback, 10)

        self.get_logger().info('OAK chessboard capture started')
        self.get_logger().info(f'image_topic: {args.image_topic}')
        self.get_logger().info(f'output_dir: {self.output_dir}')
        self.get_logger().info(f'pattern_size inner corners: {self.pattern_size}')
        self.get_logger().info(f'auto: {args.auto}, max_images: {args.max_images}')
        if args.show:
            self.get_logger().info('OpenCV window enabled. Press s to save, q to quit.')

    def _find_corners(self, gray):
        # Prefer the newer and more robust SB detector when available.
        if hasattr(cv2, 'findChessboardCornersSB'):
            flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            ok, corners = cv2.findChessboardCornersSB(gray, self.pattern_size, flags)
            if ok and corners is not None:
                return True, corners.astype(np.float32)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not ok or corners is None:
            return False, None
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return True, corners

    def _sharpness(self, gray):
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def _should_auto_save(self, corners, gray):
        now = time.time()
        if now - self.last_save_time < self.args.min_interval:
            return False
        if self.saved_count >= self.args.max_images:
            return False

        sharp = self._sharpness(gray)
        if sharp < self.args.min_sharpness:
            self.get_logger().info(f'chessboard found but image is blurry, sharpness={sharp:.1f}')
            return False

        pts = corners.reshape(-1, 2)
        center = pts.mean(axis=0)
        x, y, w, h = cv2.boundingRect(pts.astype(np.float32))
        area = float(w * h)

        if self.last_center is None:
            self.last_center = center
            self.last_area = area
            return True

        move = float(np.linalg.norm(center - self.last_center))
        area_change = abs(area - self.last_area) / max(self.last_area, 1.0)

        # Save when the board moved enough or changed apparent scale.
        if move >= self.args.min_move_px or area_change >= self.args.min_area_change:
            self.last_center = center
            self.last_area = area
            return True

        return False

    def _save_image(self, frame):
        self.saved_count += 1
        self.last_save_time = time.time()
        filename = self.output_dir / f'calib_{self.saved_count:03d}.png'
        cv2.imwrite(str(filename), frame)
        self.get_logger().info(f'saved {filename} ({self.saved_count}/{self.args.max_images})')
        if self.saved_count >= self.args.max_images:
            self.get_logger().info('Reached max_images. Press Ctrl+C to exit, or q if show window is enabled.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge convert failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = self._find_corners(gray)

        vis = frame.copy()
        if ok:
            cv2.drawChessboardCorners(vis, self.pattern_size, corners, ok)
            sharp = self._sharpness(gray)
            cv2.putText(vis, f'FOUND sharp={sharp:.1f} saved={self.saved_count}', (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2, cv2.LINE_AA)
            if self.args.auto and self._should_auto_save(corners, gray):
                self._save_image(frame)
        else:
            cv2.putText(vis, f'NOT FOUND saved={self.saved_count}', (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 2, cv2.LINE_AA)

        if self.args.show:
            cv2.imshow('OAK RGB chessboard capture', vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and ok:
                self._save_image(frame)
            elif key == ord('q'):
                rclpy.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description='Capture chessboard images from a ROS2 image topic.')
    parser.add_argument('--image-topic', default='/oak/rgb/image_raw')
    parser.add_argument('--output-dir', default='/host-home/calib_oak_rgb_1080p')
    parser.add_argument('--cols', type=int, default=9, help='inner corners per chessboard row')
    parser.add_argument('--rows', type=int, default=6, help='inner corners per chessboard column')
    parser.add_argument('--square-mm', type=float, default=25.0, help='only recorded for reference')
    parser.add_argument('--auto', action='store_true', help='auto-save frames when chessboard is detected and pose changes')
    parser.add_argument('--show', action='store_true', help='show OpenCV preview window')
    parser.add_argument('--max-images', type=int, default=40)
    parser.add_argument('--min-interval', type=float, default=0.8)
    parser.add_argument('--min-move-px', type=float, default=80.0)
    parser.add_argument('--min-area-change', type=float, default=0.08)
    parser.add_argument('--min-sharpness', type=float, default=60.0)
    args, ros_args = parser.parse_known_args()
    return args, ros_args


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = ChessboardCaptureNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
