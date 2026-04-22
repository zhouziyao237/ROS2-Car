#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


def rigid_transform_2d(src: np.ndarray, dst: np.ndarray):
    """
    求解二维刚体变换:
        dst ~= R @ src + t
    src, dst shape=(N,2)
    """
    if src.shape[0] < 3 or dst.shape[0] < 3:
        return None, None

    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)

    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    H = src_centered.T @ dst_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1.0
        R = Vt.T @ U.T

    t = dst_centroid - R @ src_centroid
    return R, t


class SimplePoseEKF:
    """
    最小三维状态 EKF:
      state = [x, y, yaw]^T
    """
    def __init__(self):
        self.x = np.zeros((3, 1), dtype=np.float64)
        self.P = np.eye(3, dtype=np.float64) * 0.1

        self.Q = np.diag([0.0008, 0.0008, 0.0005]).astype(np.float64)
        self.R = np.diag([0.06, 0.06, 0.04]).astype(np.float64)

        self.initialized = False

    def set_noise(self, qx, qy, qyaw, rx, ry, ryaw):
        self.Q = np.diag([qx, qy, qyaw]).astype(np.float64)
        self.R = np.diag([rx, ry, ryaw]).astype(np.float64)

    def predict(self):
        F = np.eye(3, dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        self.x[2, 0] = normalize_angle(self.x[2, 0])

    def update(self, zx, zy, zyaw):
        z = np.array([[zx], [zy], [zyaw]], dtype=np.float64)

        if not self.initialized:
            self.x = z.copy()
            self.x[2, 0] = normalize_angle(self.x[2, 0])
            self.initialized = True
            return

        H = np.eye(3, dtype=np.float64)
        y = z - H @ self.x
        y[2, 0] = normalize_angle(y[2, 0])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(3, dtype=np.float64) - K @ H) @ self.P
        self.x[2, 0] = normalize_angle(self.x[2, 0])

    def get_state(self):
        return float(self.x[0, 0]), float(self.x[1, 0]), float(self.x[2, 0])


class ScanMatchOdom(Node):
    def __init__(self):
        super().__init__('scan_match_odom')

        # topics / frames
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # scan filter
        self.declare_parameter('min_range', 0.18)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('downsample_step', 5)

        # icp / matching
        self.declare_parameter('icp_max_iterations', 8)
        self.declare_parameter('icp_max_corr_dist', 0.20)
        self.declare_parameter('min_correspondences', 30)
        self.declare_parameter('reject_mean_err_above', 0.08)

        # motion gate
        self.declare_parameter('min_motion_trans', 0.004)
        self.declare_parameter('min_motion_rot', 0.004)
        self.declare_parameter('static_lock_trans', 0.006)
        self.declare_parameter('static_lock_rot', 0.006)
        self.declare_parameter('reject_trans_above', 0.08)
        self.declare_parameter('reject_rot_above', 0.12)

        # ekf
        self.declare_parameter('use_ekf', True)
        self.declare_parameter('ekf_qx', 0.0008)
        self.declare_parameter('ekf_qy', 0.0008)
        self.declare_parameter('ekf_qyaw', 0.0005)
        self.declare_parameter('ekf_rx', 0.06)
        self.declare_parameter('ekf_ry', 0.06)
        self.declare_parameter('ekf_ryaw', 0.04)

        # velocity smoothing
        self.declare_parameter('vel_alpha', 0.2)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.downsample_step = int(self.get_parameter('downsample_step').value)

        self.icp_max_iterations = int(self.get_parameter('icp_max_iterations').value)
        self.icp_max_corr_dist = float(self.get_parameter('icp_max_corr_dist').value)
        self.min_correspondences = int(self.get_parameter('min_correspondences').value)
        self.reject_mean_err_above = float(self.get_parameter('reject_mean_err_above').value)

        self.min_motion_trans = float(self.get_parameter('min_motion_trans').value)
        self.min_motion_rot = float(self.get_parameter('min_motion_rot').value)
        self.static_lock_trans = float(self.get_parameter('static_lock_trans').value)
        self.static_lock_rot = float(self.get_parameter('static_lock_rot').value)
        self.reject_trans_above = float(self.get_parameter('reject_trans_above').value)
        self.reject_rot_above = float(self.get_parameter('reject_rot_above').value)

        self.use_ekf = bool(self.get_parameter('use_ekf').value)
        self.vel_alpha = float(self.get_parameter('vel_alpha').value)

        self.ekf = SimplePoseEKF()
        self.ekf.set_noise(
            float(self.get_parameter('ekf_qx').value),
            float(self.get_parameter('ekf_qy').value),
            float(self.get_parameter('ekf_qyaw').value),
            float(self.get_parameter('ekf_rx').value),
            float(self.get_parameter('ekf_ry').value),
            float(self.get_parameter('ekf_ryaw').value),
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.prev_points = None
        self.last_stamp = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_pub_x = 0.0
        self.last_pub_y = 0.0
        self.last_pub_yaw = 0.0
        self.last_pub_time = None

        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0

        self.get_logger().info(
            f'Listening {self.scan_topic}, publishing {self.odom_topic}, TF {self.odom_frame}->{self.base_frame}'
        )

    def scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.array(msg.ranges, dtype=np.float64)
        n = ranges.shape[0]
        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment

        valid = np.isfinite(ranges)
        valid &= (ranges >= self.min_range)
        valid &= (ranges <= self.max_range)

        ranges = ranges[valid]
        angles = angles[valid]

        if ranges.size == 0:
            return np.empty((0, 2), dtype=np.float64)

        step = max(1, self.downsample_step)
        ranges = ranges[::step]
        angles = angles[::step]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        pts = np.stack([x, y], axis=1)
        return pts.astype(np.float64)

    def nearest_neighbor(self, src: np.ndarray, dst: np.ndarray):
        if src.shape[0] == 0 or dst.shape[0] == 0:
            return None, None, None

        diff = src[:, None, :] - dst[None, :, :]
        d2 = np.sum(diff * diff, axis=2)
        nn_idx = np.argmin(d2, axis=1)
        nn_d = np.sqrt(d2[np.arange(src.shape[0]), nn_idx])

        matched_src = src
        matched_dst = dst[nn_idx]
        return matched_src, matched_dst, nn_d

    def icp_relative_motion(self, prev_pts: np.ndarray, curr_pts: np.ndarray):
        """
        求环境点从 prev 到 curr 的变换，再反推机器人运动。
        """
        if prev_pts.shape[0] < self.min_correspondences or curr_pts.shape[0] < self.min_correspondences:
            return None

        R_total = np.eye(2, dtype=np.float64)
        t_total = np.zeros(2, dtype=np.float64)

        src = prev_pts.copy()
        dst = curr_pts.copy()

        mean_err = 1e9

        for _ in range(self.icp_max_iterations):
            src_transformed = (R_total @ src.T).T + t_total

            matched_src, matched_dst, d = self.nearest_neighbor(src_transformed, dst)
            if matched_src is None:
                return None

            mask = d < self.icp_max_corr_dist
            if np.count_nonzero(mask) < self.min_correspondences:
                return None

            src_corr = matched_src[mask]
            dst_corr = matched_dst[mask]

            R_inc, t_inc = rigid_transform_2d(src_corr, dst_corr)
            if R_inc is None:
                return None

            R_total = R_inc @ R_total
            t_total = R_inc @ t_total + t_inc

            mean_err = float(np.mean(d[mask]))

        # 点云坐标变换取逆 -> 机器人运动
        R_robot = R_total.T
        t_robot = -R_robot @ t_total
        dyaw = math.atan2(R_robot[1, 0], R_robot[0, 0])

        dx = float(t_robot[0])
        dy = float(t_robot[1])
        return dx, dy, dyaw, mean_err

    def publish_odom_and_tf(self, stamp, x, y, yaw, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quat(yaw)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance = [
            0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.03, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)
        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08
        ]

        self.odom_pub.publish(odom)

        if self.publish_tf and self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(x)
            tf_msg.transform.translation.y = float(y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

    def scan_callback(self, msg: LaserScan):
        points = self.scan_to_points(msg)
        if points.shape[0] < self.min_correspondences:
            return

        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if self.prev_points is None:
            self.prev_points = points
            self.last_stamp = stamp_sec
            self.last_pub_time = stamp_sec
            self.publish_odom_and_tf(stamp, self.x, self.y, self.yaw, 0.0, 0.0, 0.0)
            return

        dt = stamp_sec - self.last_stamp
        if dt <= 1e-6 or not math.isfinite(dt):
            self.prev_points = points
            self.last_stamp = stamp_sec
            return

        result = self.icp_relative_motion(self.prev_points, points)
        self.prev_points = points
        self.last_stamp = stamp_sec

        if result is None:
            return

        dx_body, dy_body, dyaw, mean_err = result

        # 匹配误差太大直接丢
        if (not math.isfinite(mean_err)) or (mean_err > self.reject_mean_err_above):
            return

        # 小抖动压零
        if abs(dx_body) < self.min_motion_trans:
            dx_body = 0.0
        if abs(dy_body) < self.min_motion_trans:
            dy_body = 0.0
        if abs(dyaw) < self.min_motion_rot:
            dyaw = 0.0

        trans_norm = math.hypot(dx_body, dy_body)

        # 静止锁定
        if trans_norm < self.static_lock_trans and abs(dyaw) < self.static_lock_rot:
            dx_body = 0.0
            dy_body = 0.0
            dyaw = 0.0
            trans_norm = 0.0

        # 大跳变拒绝
        if trans_norm > self.reject_trans_above or abs(dyaw) > self.reject_rot_above:
            self.get_logger().warn(
                f'Reject ICP jump: trans={trans_norm:.3f}, rot={dyaw:.3f}, err={mean_err:.3f}'
            )
            return

        # body 增量 -> world pose
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        dx_world = cos_y * dx_body - sin_y * dy_body
        dy_world = sin_y * dx_body + cos_y * dy_body

        raw_x = self.x + dx_world
        raw_y = self.y + dy_world
        raw_yaw = normalize_angle(self.yaw + dyaw)

        if self.use_ekf:
            self.ekf.predict()
            self.ekf.update(raw_x, raw_y, raw_yaw)
            self.x, self.y, self.yaw = self.ekf.get_state()
        else:
            self.x, self.y, self.yaw = raw_x, raw_y, raw_yaw

        pub_dt = stamp_sec - self.last_pub_time if self.last_pub_time is not None else dt
        if pub_dt <= 1e-6:
            pub_dt = dt

        raw_vx = (self.x - self.last_pub_x) / pub_dt
        raw_vy = (self.y - self.last_pub_y) / pub_dt
        raw_wz = normalize_angle(self.yaw - self.last_pub_yaw) / pub_dt

        a = self.vel_alpha
        self.filtered_vx = (1.0 - a) * self.filtered_vx + a * raw_vx
        self.filtered_vy = (1.0 - a) * self.filtered_vy + a * raw_vy
        self.filtered_wz = (1.0 - a) * self.filtered_wz + a * raw_wz

        vx = self.filtered_vx
        vy = self.filtered_vy
        wz = self.filtered_wz

        self.publish_odom_and_tf(stamp, self.x, self.y, self.yaw, vx, vy, wz)

        self.last_pub_x = self.x
        self.last_pub_y = self.y
        self.last_pub_yaw = self.yaw
        self.last_pub_time = stamp_sec


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatchOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()