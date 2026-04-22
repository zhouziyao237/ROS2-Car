#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanMatchOdom(Node):
    def __init__(self) -> None:
        super().__init__('scan_match_odom')
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.min_range = float(self.declare_parameter('min_range', 0.08).value)
        self.max_range = float(self.declare_parameter('max_range', 20.0).value)
        self.max_points = int(self.declare_parameter('max_points', 180).value)
        self.icp_iterations = int(self.declare_parameter('icp_iterations', 8).value)
        self.max_corr_dist = float(self.declare_parameter('max_corr_dist', 0.45).value)
        self.downsample_step = int(self.declare_parameter('downsample_step', 6).value)
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        self.prev_pts: Optional[np.ndarray] = None
        self.prev_stamp = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_dx = 0.0
        self.last_dy = 0.0
        self.last_dyaw = 0.0
        self.last_dt = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_pub = TransformBroadcaster(self)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 20)

        self.get_logger().info(
            f'lidar odom started. scan_topic={self.scan_topic}, odom_topic={self.odom_topic}, '
            f'frames {self.odom_frame}->{self.base_frame}'
        )

    def scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float64) * msg.angle_increment

        mask = np.isfinite(ranges)
        mask &= ranges >= self.min_range
        mask &= ranges <= self.max_range
        if not np.any(mask):
            return np.empty((0, 2), dtype=np.float64)

        ranges = ranges[mask]
        angles = angles[mask]

        if self.downsample_step > 1:
            ranges = ranges[:: self.downsample_step]
            angles = angles[:: self.downsample_step]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        pts = np.stack([x, y], axis=1)

        if pts.shape[0] > self.max_points:
            idx = np.linspace(0, pts.shape[0] - 1, self.max_points).astype(np.int32)
            pts = pts[idx]

        return pts

    def nearest_pairs(self, src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if src.size == 0 or dst.size == 0:
            return np.empty((0, 2)), np.empty((0, 2))
        # Brute-force NN on downsampled points. Fine for baseline/demo.
        d2 = np.sum((src[:, None, :] - dst[None, :, :]) ** 2, axis=2)
        nn_idx = np.argmin(d2, axis=1)
        nn_d = np.sqrt(d2[np.arange(src.shape[0]), nn_idx])
        keep = nn_d < self.max_corr_dist
        return src[keep], dst[nn_idx[keep]]

    def fit_transform(self, src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if src.shape[0] < 3:
            return np.eye(2), np.zeros(2)
        src_mean = src.mean(axis=0)
        dst_mean = dst.mean(axis=0)
        src_c = src - src_mean
        dst_c = dst - dst_mean
        H = src_c.T @ dst_c
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = dst_mean - R @ src_mean
        return R, t

    def icp(self, current_pts: np.ndarray, previous_pts: np.ndarray) -> Tuple[float, float, float]:
        if current_pts.shape[0] < 10 or previous_pts.shape[0] < 10:
            return 0.0, 0.0, 0.0

        src = current_pts.copy()
        R_total = np.eye(2)
        t_total = np.zeros(2)

        for _ in range(self.icp_iterations):
            src_match, dst_match = self.nearest_pairs(src, previous_pts)
            if src_match.shape[0] < 8:
                break
            R, t = self.fit_transform(src_match, dst_match)
            src = (R @ src.T).T + t
            R_total = R @ R_total
            t_total = R @ t_total + t

        dyaw = math.atan2(R_total[1, 0], R_total[0, 0])
        dx = float(t_total[0])
        dy = float(t_total[1])
        return dx, dy, dyaw

    def publish_odom(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        if self.last_dt > 1e-6:
            odom.twist.twist.linear.x = self.last_dx / self.last_dt
            odom.twist.twist.linear.y = self.last_dy / self.last_dt
            odom.twist.twist.angular.z = self.last_dyaw / self.last_dt

        # Conservative covariances for a baseline scan-matching odom.
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.08
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = math.sin(self.yaw / 2.0)
            tf_msg.transform.rotation.w = math.cos(self.yaw / 2.0)
            self.tf_pub.sendTransform(tf_msg)

    def scan_cb(self, msg: LaserScan) -> None:
        pts = self.scan_to_points(msg)
        if pts.shape[0] < 10:
            self.get_logger().warn('Too few valid scan points for lidar odom.')
            return

        if self.prev_pts is None:
            self.prev_pts = pts
            self.prev_stamp = msg.header.stamp
            self.publish_odom(msg.header.stamp)
            return

        dx_local, dy_local, dyaw = self.icp(pts, self.prev_pts)

        # Convert motion expressed in previous robot frame into odom/world.
        dx_world = math.cos(self.yaw) * dx_local - math.sin(self.yaw) * dy_local
        dy_world = math.sin(self.yaw) * dx_local + math.cos(self.yaw) * dy_local

        self.x += dx_world
        self.y += dy_world
        self.yaw = wrap_angle(self.yaw + dyaw)

        sec = float(msg.header.stamp.sec - self.prev_stamp.sec)
        nsec = float(msg.header.stamp.nanosec - self.prev_stamp.nanosec) * 1e-9
        self.last_dt = max(sec + nsec, 1e-6)
        self.last_dx = dx_world
        self.last_dy = dy_world
        self.last_dyaw = dyaw

        self.prev_pts = pts
        self.prev_stamp = msg.header.stamp
        self.publish_odom(msg.header.stamp)


def main() -> None:
    rclpy.init()
    node = ScanMatchOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
