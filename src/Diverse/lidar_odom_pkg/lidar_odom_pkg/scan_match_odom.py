#!/usr/bin/env python3
"""
ROS2 2D LiDAR scan-matching odometry.

Core design:
  1. LaserScan -> filtered 2D points in base frame.
  2. Constant-velocity Kalman prediction gives an initial SE(2) guess.
  3. Point-to-point ICP aligns consecutive scans.
  4. Geometry/observability gating suppresses unreliable lateral/yaw updates.
  5. Dynamic covariance is published for later fusion with robot_localization/Nav2.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


Array2 = np.ndarray


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_to_quat(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def rot2(theta: float) -> Array2:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def rigid_transform_2d(src: Array2, dst: Array2) -> Tuple[Optional[Array2], Optional[Array2]]:
    """Least-squares SE(2) transform: dst ~= R @ src + t."""
    if src.shape[0] < 3 or dst.shape[0] < 3:
        return None, None

    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)
    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    H = src_centered.T @ dst_centered
    try:
        U, _, Vt = np.linalg.svd(H)
    except np.linalg.LinAlgError:
        return None, None

    R = Vt.T @ U.T
    if np.linalg.det(R) < 0.0:
        Vt[1, :] *= -1.0
        R = Vt.T @ U.T

    t = dst_centroid - R @ src_centroid
    return R, t


def robot_delta_to_env_transform(dx: float, dy: float, dyaw: float) -> Tuple[Array2, Array2]:
    """
    Robot body delta from previous base to current base -> transform old scan points
    into current scan frame. This is the inverse transform used by scan matching.
    """
    R_robot = rot2(dyaw)
    t_robot = np.array([dx, dy], dtype=np.float64)
    R_env = R_robot.T
    t_env = -R_env @ t_robot
    return R_env, t_env


def env_transform_to_robot_delta(R_env: Array2, t_env: Array2) -> Tuple[float, float, float]:
    """Inverse of robot_delta_to_env_transform."""
    R_robot = R_env.T
    t_robot = -R_robot @ t_env
    dyaw = math.atan2(R_robot[1, 0], R_robot[0, 0])
    return float(t_robot[0]), float(t_robot[1]), normalize_angle(dyaw)


class ConstantVelocityPoseKF:
    """
    6-state constant-velocity Kalman filter.

    State: [x, y, yaw, vx_world, vy_world, wz]^T
    Measurement: [x, y, yaw]^T from scan matching.
    """

    def __init__(self) -> None:
        self.x = np.zeros((6, 1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 0.05
        self.initialized = False

        self.q_pos = 0.003
        self.q_yaw = 0.002
        self.q_vel = 0.08
        self.q_wz = 0.05

        self.r_x = 0.03
        self.r_y = 0.04
        self.r_yaw = 0.04

    def set_noise(
        self,
        q_pos: float,
        q_yaw: float,
        q_vel: float,
        q_wz: float,
        r_x: float,
        r_y: float,
        r_yaw: float,
    ) -> None:
        self.q_pos = max(float(q_pos), 1e-12)
        self.q_yaw = max(float(q_yaw), 1e-12)
        self.q_vel = max(float(q_vel), 1e-12)
        self.q_wz = max(float(q_wz), 1e-12)
        self.r_x = max(float(r_x), 1e-12)
        self.r_y = max(float(r_y), 1e-12)
        self.r_yaw = max(float(r_yaw), 1e-12)

    def initialize(self, x: float, y: float, yaw: float) -> None:
        self.x[:] = 0.0
        self.x[0, 0] = float(x)
        self.x[1, 0] = float(y)
        self.x[2, 0] = normalize_angle(float(yaw))
        self.P = np.eye(6, dtype=np.float64) * 0.05
        self.initialized = True

    def predict_copy(self, dt: float) -> Tuple[Array2, Array2]:
        if not self.initialized:
            return self.x.copy(), self.P.copy()

        dt = max(float(dt), 1e-6)
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Simple white-noise process model. dt scaling prevents over-confidence at high rate.
        q_pos = self.q_pos * max(dt, 1e-3)
        q_yaw = self.q_yaw * max(dt, 1e-3)
        q_vel = self.q_vel * max(dt, 1e-3)
        q_wz = self.q_wz * max(dt, 1e-3)
        Q = np.diag([q_pos, q_pos, q_yaw, q_vel, q_vel, q_wz]).astype(np.float64)

        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q
        x_pred[2, 0] = normalize_angle(float(x_pred[2, 0]))
        return x_pred, P_pred

    def commit_prediction(self, x_pred: Array2, p_pred: Array2) -> None:
        self.x = x_pred.copy()
        self.P = p_pred.copy()
        self.x[2, 0] = normalize_angle(float(self.x[2, 0]))

    def update_pose(self, zx: float, zy: float, zyaw: float, r_diag: Optional[Sequence[float]] = None) -> None:
        if not self.initialized:
            self.initialize(zx, zy, zyaw)
            return

        z = np.array([[zx], [zy], [normalize_angle(zyaw)]], dtype=np.float64)
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        if r_diag is None:
            Rm = np.diag([self.r_x, self.r_y, self.r_yaw]).astype(np.float64)
        else:
            Rm = np.diag([max(float(r_diag[0]), 1e-9), max(float(r_diag[1]), 1e-9), max(float(r_diag[2]), 1e-9)]).astype(np.float64)

        y = z - H @ self.x
        y[2, 0] = normalize_angle(float(y[2, 0]))

        S = H @ self.P @ H.T + Rm
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float64)
        # Joseph form is numerically safer than (I-KH)P.
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ Rm @ K.T
        self.x[2, 0] = normalize_angle(float(self.x[2, 0]))

    def blend_velocity(self, vx_world: float, vy_world: float, wz: float, alpha: float) -> None:
        if not self.initialized:
            return
        a = clamp(float(alpha), 0.0, 1.0)
        self.x[3, 0] = (1.0 - a) * self.x[3, 0] + a * float(vx_world)
        self.x[4, 0] = (1.0 - a) * self.x[4, 0] + a * float(vy_world)
        self.x[5, 0] = (1.0 - a) * self.x[5, 0] + a * float(wz)

    def get_pose(self) -> Tuple[float, float, float]:
        return float(self.x[0, 0]), float(self.x[1, 0]), normalize_angle(float(self.x[2, 0]))

    def get_world_velocity(self) -> Tuple[float, float, float]:
        return float(self.x[3, 0]), float(self.x[4, 0]), float(self.x[5, 0])


class ScanMatchOdom(Node):
    def __init__(self) -> None:
        super().__init__('scan_match_odom')

        # topics / frames
        self.declare_parameter('scan_topic', '/scan_front')
        self.declare_parameter('odom_topic', '/lidar/odom')
        self.declare_parameter('diagnostics_topic', '/lidar/odom_debug')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        # scan filter and extrinsic. Angles are degrees in original LaserScan frame.
        self.declare_parameter('min_range', 0.18)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('downsample_step', 3)
        self.declare_parameter('use_angle_filter', True)
        self.declare_parameter('angle_windows_deg', [-95.0, 95.0])
        self.declare_parameter('scan_yaw_offset_deg', 0.0)
        self.declare_parameter('scan_x_offset', 0.0)
        self.declare_parameter('scan_y_offset', 0.0)
        self.declare_parameter('invert_scan_x', False)
        self.declare_parameter('invert_scan_y', False)

        # ICP / matching
        self.declare_parameter('icp_max_iterations', 12)
        self.declare_parameter('icp_max_corr_dist', 0.12)
        self.declare_parameter('icp_trim_ratio', 0.85)
        self.declare_parameter('icp_converge_trans', 0.0002)
        self.declare_parameter('icp_converge_rot', 0.0003)
        self.declare_parameter('min_correspondences', 50)
        self.declare_parameter('reject_mean_err_above', 0.06)

        # motion gate
        self.declare_parameter('min_motion_trans', 0.002)
        self.declare_parameter('min_motion_rot', 0.003)
        self.declare_parameter('static_lock_trans', 0.003)
        self.declare_parameter('static_lock_rot', 0.003)
        self.declare_parameter('reject_trans_above', 0.12)
        self.declare_parameter('reject_rot_above', 0.18)
        self.declare_parameter('max_dt', 0.25)

        # prediction / Kalman
        self.declare_parameter('use_kalman_predict', True)
        self.declare_parameter('use_kalman_update', True)
        self.declare_parameter('kalman_q_pos', 0.003)
        self.declare_parameter('kalman_q_yaw', 0.002)
        self.declare_parameter('kalman_q_vel', 0.08)
        self.declare_parameter('kalman_q_wz', 0.05)
        self.declare_parameter('kalman_r_x', 0.03)
        self.declare_parameter('kalman_r_y', 0.04)
        self.declare_parameter('kalman_r_yaw', 0.04)
        self.declare_parameter('velocity_feedback_alpha', 0.35)

        # External motion prior for fast motion.
        # Use wheel/IMU/cmd_vel to seed ICP and optionally deskew one LaserScan frame.
        self.declare_parameter('use_external_twist_prior', False)
        self.declare_parameter('prior_source', 'auto')  # auto / wheel_imu / wheel / cmd_vel / internal
        self.declare_parameter('external_odom_topic', '/wheel/odom')
        self.declare_parameter('external_imu_topic', '/imu/data')
        self.declare_parameter('external_cmd_vel_topic', '/cmd_vel_selected')
        self.declare_parameter('control_mode_topic', '/control_mode')
        self.declare_parameter('chassis_control_mode_topic', '/chassis_control_mode')
        self.declare_parameter('chassis_control_mode_timeout', 1.0)
        self.declare_parameter('control_mode_default', 'manual')
        self.declare_parameter('manual_prior_source', 'internal')
        self.declare_parameter('manual_disable_scan_deskew', True)
        self.declare_parameter('external_twist_timeout', 0.35)
        self.declare_parameter('external_prior_weight', 0.90)
        self.declare_parameter('use_scan_deskew', False)
        self.declare_parameter('deskew_center_scan', True)
        self.declare_parameter('deskew_max_time', 0.12)

        # adaptive observability-aware motion constraint
        self.declare_parameter('use_adaptive_constraint', True)
        self.declare_parameter('sigma_ref_x', 0.020)
        self.declare_parameter('sigma_ref_y', 0.012)
        self.declare_parameter('sigma_ref_yaw', 0.020)
        self.declare_parameter('alpha_x_min', 0.60)
        self.declare_parameter('alpha_x_max', 1.00)
        self.declare_parameter('alpha_y_min', 0.00)
        self.declare_parameter('alpha_y_max', 1.00)
        self.declare_parameter('alpha_yaw_min', 0.20)
        self.declare_parameter('alpha_yaw_max', 1.00)
        self.declare_parameter('dy_prior', 0.0)
        self.declare_parameter('dyaw_prior', 0.0)
        self.declare_parameter('use_scan_geometry_gate', True)
        self.declare_parameter('geometry_sector_deadzone_deg', 8.0)
        self.declare_parameter('geometry_weight_y', 0.60)
        self.declare_parameter('geometry_weight_yaw', 0.55)

        # covariance for downstream fusion
        self.declare_parameter('publish_dynamic_covariance', True)
        self.declare_parameter('min_cov_x', 0.002)
        self.declare_parameter('min_cov_y', 0.004)
        self.declare_parameter('min_cov_yaw', 0.006)
        self.declare_parameter('max_cov_x', 0.20)
        self.declare_parameter('max_cov_y', 0.30)
        self.declare_parameter('max_cov_yaw', 0.30)

        # velocity smoothing and debug
        self.declare_parameter('vel_alpha', 0.20)
        self.declare_parameter('debug_log_period', 20)
        self.declare_parameter('publish_debug', True)

        self.load_parameters()

        self.kf = ConstantVelocityPoseKF()
        self.kf.set_noise(
            self.kalman_q_pos,
            self.kalman_q_yaw,
            self.kalman_q_vel,
            self.kalman_q_wz,
            self.kalman_r_x,
            self.kalman_r_y,
            self.kalman_r_yaw,
        )

        # Latest external motion prior in base_link coordinates.
        # Odometry/Twist linear.x/y are assumed to be body-frame velocities, as nav_msgs/Odometry twist
        # is conventionally expressed in child_frame_id. IMU contributes angular_velocity.z.
        self.prior_odom_vx = 0.0
        self.prior_odom_vy = 0.0
        self.prior_odom_wz = 0.0
        self.prior_odom_stamp = -1e30
        self.prior_cmd_vx = 0.0
        self.prior_cmd_vy = 0.0
        self.prior_cmd_wz = 0.0
        self.prior_cmd_stamp = -1e30
        self.prior_imu_wz = 0.0
        self.prior_imu_stamp = -1e30
        self.control_mode = self.control_mode_default
        self.chassis_control_mode = None
        self.chassis_control_mode_stamp = -1e30
        self.last_deskew_source = 'none'
        self.last_used_prior_source = 'internal'
        self.last_used_prior_twist = (0.0, 0.0, 0.0)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.debug_pub = self.create_publisher(String, self.diagnostics_topic, 10) if self.publish_debug else None
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        if self.use_external_twist_prior or self.use_scan_deskew:
            self.external_odom_sub = self.create_subscription(
                Odometry, self.external_odom_topic, self.external_odom_callback, 20
            ) if self.external_odom_topic else None
            self.external_imu_sub = self.create_subscription(
                Imu, self.external_imu_topic, self.external_imu_callback, qos_profile_sensor_data
            ) if self.external_imu_topic else None
            self.external_cmd_sub = self.create_subscription(
                Twist, self.external_cmd_vel_topic, self.external_cmd_vel_callback, 20
            ) if self.external_cmd_vel_topic else None
        else:
            self.external_odom_sub = None
            self.external_imu_sub = None
            self.external_cmd_sub = None
        self.control_mode_sub = self.create_subscription(
            String, self.control_mode_topic, self.control_mode_callback, 10
        ) if self.control_mode_topic else None
        self.chassis_control_mode_sub = self.create_subscription(
            String, self.chassis_control_mode_topic, self.chassis_control_mode_callback, 10
        ) if self.chassis_control_mode_topic else None

        self.prev_points: Optional[Array2] = None
        self.prev_base_angles: Optional[Array2] = None
        self.last_stamp: Optional[float] = None
        self.last_pub_time: Optional[float] = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_pub_x = 0.0
        self.last_pub_y = 0.0
        self.last_pub_yaw = 0.0

        self.filtered_vx_body = 0.0
        self.filtered_vy_body = 0.0
        self.filtered_wz = 0.0

        self.last_num_corr = 0
        self.last_match_err = 0.0
        self.last_sigmas = (0.05, 0.05, 0.05)
        self.last_alphas = (1.0, 1.0, 1.0)
        self.last_scan_gate = (1.0, 1.0)
        self.last_cond = 1.0
        self.callback_count = 0

        self.get_logger().info(
            f'Listening {self.scan_topic}, publishing {self.odom_topic}, '
            f'frames {self.odom_frame}->{self.base_frame}, angle_windows_deg={self.angle_windows_deg}, '
            f'scan_yaw_offset_deg={math.degrees(self.scan_yaw_offset):.1f}, '
            f'prior_source={self.prior_source}, deskew={self.use_scan_deskew}, '
            f'control_mode={self.control_mode}'
        )

    def load_parameters(self) -> None:
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.diagnostics_topic = str(self.get_parameter('diagnostics_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.downsample_step = max(1, int(self.get_parameter('downsample_step').value))
        self.use_angle_filter = bool(self.get_parameter('use_angle_filter').value)
        self.angle_windows_deg = [float(v) for v in self.get_parameter('angle_windows_deg').value]
        if len(self.angle_windows_deg) % 2 != 0:
            self.get_logger().warn('angle_windows_deg length must be even. Last value will be ignored.')
            self.angle_windows_deg = self.angle_windows_deg[:-1]
        self.angle_windows = [(math.radians(self.angle_windows_deg[i]), math.radians(self.angle_windows_deg[i + 1])) for i in range(0, len(self.angle_windows_deg), 2)]
        self.scan_yaw_offset = math.radians(float(self.get_parameter('scan_yaw_offset_deg').value))
        self.scan_x_offset = float(self.get_parameter('scan_x_offset').value)
        self.scan_y_offset = float(self.get_parameter('scan_y_offset').value)
        self.invert_scan_x = bool(self.get_parameter('invert_scan_x').value)
        self.invert_scan_y = bool(self.get_parameter('invert_scan_y').value)

        self.icp_max_iterations = int(self.get_parameter('icp_max_iterations').value)
        self.icp_max_corr_dist = float(self.get_parameter('icp_max_corr_dist').value)
        self.icp_trim_ratio = clamp(float(self.get_parameter('icp_trim_ratio').value), 0.20, 1.0)
        self.icp_converge_trans = float(self.get_parameter('icp_converge_trans').value)
        self.icp_converge_rot = float(self.get_parameter('icp_converge_rot').value)
        self.min_correspondences = int(self.get_parameter('min_correspondences').value)
        self.reject_mean_err_above = float(self.get_parameter('reject_mean_err_above').value)

        self.min_motion_trans = float(self.get_parameter('min_motion_trans').value)
        self.min_motion_rot = float(self.get_parameter('min_motion_rot').value)
        self.static_lock_trans = float(self.get_parameter('static_lock_trans').value)
        self.static_lock_rot = float(self.get_parameter('static_lock_rot').value)
        self.reject_trans_above = float(self.get_parameter('reject_trans_above').value)
        self.reject_rot_above = float(self.get_parameter('reject_rot_above').value)
        self.max_dt = float(self.get_parameter('max_dt').value)

        self.use_kalman_predict = bool(self.get_parameter('use_kalman_predict').value)
        self.use_kalman_update = bool(self.get_parameter('use_kalman_update').value)
        self.kalman_q_pos = float(self.get_parameter('kalman_q_pos').value)
        self.kalman_q_yaw = float(self.get_parameter('kalman_q_yaw').value)
        self.kalman_q_vel = float(self.get_parameter('kalman_q_vel').value)
        self.kalman_q_wz = float(self.get_parameter('kalman_q_wz').value)
        self.kalman_r_x = float(self.get_parameter('kalman_r_x').value)
        self.kalman_r_y = float(self.get_parameter('kalman_r_y').value)
        self.kalman_r_yaw = float(self.get_parameter('kalman_r_yaw').value)
        self.velocity_feedback_alpha = float(self.get_parameter('velocity_feedback_alpha').value)

        self.use_external_twist_prior = bool(self.get_parameter('use_external_twist_prior').value)
        self.prior_source = str(self.get_parameter('prior_source').value).strip().lower()
        self.external_odom_topic = str(self.get_parameter('external_odom_topic').value)
        self.external_imu_topic = str(self.get_parameter('external_imu_topic').value)
        self.external_cmd_vel_topic = str(self.get_parameter('external_cmd_vel_topic').value)
        self.control_mode_topic = str(self.get_parameter('control_mode_topic').value)
        self.chassis_control_mode_topic = str(self.get_parameter('chassis_control_mode_topic').value)
        self.chassis_control_mode_timeout = max(
            0.02, float(self.get_parameter('chassis_control_mode_timeout').value))
        self.control_mode_default = str(self.get_parameter('control_mode_default').value).strip().lower()
        self.manual_prior_source = str(self.get_parameter('manual_prior_source').value).strip().lower()
        self.manual_disable_scan_deskew = bool(self.get_parameter('manual_disable_scan_deskew').value)
        self.external_twist_timeout = max(0.02, float(self.get_parameter('external_twist_timeout').value))
        self.external_prior_weight = clamp(float(self.get_parameter('external_prior_weight').value), 0.0, 1.0)
        self.use_scan_deskew = bool(self.get_parameter('use_scan_deskew').value)
        self.deskew_center_scan = bool(self.get_parameter('deskew_center_scan').value)
        self.deskew_max_time = max(0.0, float(self.get_parameter('deskew_max_time').value))

        self.use_adaptive_constraint = bool(self.get_parameter('use_adaptive_constraint').value)
        self.sigma_ref_x = float(self.get_parameter('sigma_ref_x').value)
        self.sigma_ref_y = float(self.get_parameter('sigma_ref_y').value)
        self.sigma_ref_yaw = float(self.get_parameter('sigma_ref_yaw').value)
        self.alpha_x_min = float(self.get_parameter('alpha_x_min').value)
        self.alpha_x_max = float(self.get_parameter('alpha_x_max').value)
        self.alpha_y_min = float(self.get_parameter('alpha_y_min').value)
        self.alpha_y_max = float(self.get_parameter('alpha_y_max').value)
        self.alpha_yaw_min = float(self.get_parameter('alpha_yaw_min').value)
        self.alpha_yaw_max = float(self.get_parameter('alpha_yaw_max').value)
        self.dy_prior = float(self.get_parameter('dy_prior').value)
        self.dyaw_prior = float(self.get_parameter('dyaw_prior').value)
        self.use_scan_geometry_gate = bool(self.get_parameter('use_scan_geometry_gate').value)
        self.geometry_sector_deadzone = math.radians(float(self.get_parameter('geometry_sector_deadzone_deg').value))
        self.geometry_weight_y = float(self.get_parameter('geometry_weight_y').value)
        self.geometry_weight_yaw = float(self.get_parameter('geometry_weight_yaw').value)

        self.publish_dynamic_covariance = bool(self.get_parameter('publish_dynamic_covariance').value)
        self.min_cov_x = float(self.get_parameter('min_cov_x').value)
        self.min_cov_y = float(self.get_parameter('min_cov_y').value)
        self.min_cov_yaw = float(self.get_parameter('min_cov_yaw').value)
        self.max_cov_x = float(self.get_parameter('max_cov_x').value)
        self.max_cov_y = float(self.get_parameter('max_cov_y').value)
        self.max_cov_yaw = float(self.get_parameter('max_cov_yaw').value)

        self.vel_alpha = clamp(float(self.get_parameter('vel_alpha').value), 0.0, 1.0)
        self.debug_log_period = max(1, int(self.get_parameter('debug_log_period').value))
        self.publish_debug = bool(self.get_parameter('publish_debug').value)

    def msg_stamp_to_sec(self, stamp_msg) -> float:
        sec = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
        if sec <= 0.0:
            return self.now_sec()
        return sec

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def external_odom_callback(self, msg: Odometry) -> None:
        self.prior_odom_vx = float(msg.twist.twist.linear.x)
        self.prior_odom_vy = float(msg.twist.twist.linear.y)
        self.prior_odom_wz = float(msg.twist.twist.angular.z)
        self.prior_odom_stamp = self.msg_stamp_to_sec(msg.header.stamp)

    def external_imu_callback(self, msg: Imu) -> None:
        self.prior_imu_wz = float(msg.angular_velocity.z)
        self.prior_imu_stamp = self.msg_stamp_to_sec(msg.header.stamp)

    def external_cmd_vel_callback(self, msg: Twist) -> None:
        self.prior_cmd_vx = float(msg.linear.x)
        self.prior_cmd_vy = float(msg.linear.y)
        self.prior_cmd_wz = float(msg.angular.z)
        self.prior_cmd_stamp = self.now_sec()

    def control_mode_callback(self, msg: String) -> None:
        mode = str(msg.data).strip().lower()
        if mode not in ('manual', 'auto'):
            return
        if mode != self.control_mode:
            self.control_mode = mode
            self.get_logger().info(
                f'Control mode switched to {self.get_effective_control_mode()}; '
                f'prior_source={self.get_effective_prior_source()}, '
                f'deskew={"off" if not self.should_use_scan_deskew() else "on"}'
            )

    def chassis_control_mode_callback(self, msg: String) -> None:
        mode = str(msg.data).strip().lower()
        if mode not in ('manual', 'auto'):
            return
        changed = (mode != self.chassis_control_mode)
        self.chassis_control_mode = mode
        self.chassis_control_mode_stamp = self.now_sec()
        if changed:
            self.get_logger().info(
                f'Chassis control mode switched to {self.chassis_control_mode}; '
                f'effective_mode={self.get_effective_control_mode()}, '
                f'prior_source={self.get_effective_prior_source()}, '
                f'deskew={"off" if not self.should_use_scan_deskew() else "on"}'
            )

    def get_effective_control_mode(self) -> str:
        if (
            self.chassis_control_mode == 'manual' and
            (self.now_sec() - self.chassis_control_mode_stamp) <= self.chassis_control_mode_timeout
        ):
            return 'manual'
        return self.control_mode

    def is_prior_fresh(self, stamp_sec: float, prior_stamp: float) -> bool:
        if prior_stamp < -1e20:
            return False
        return abs(float(stamp_sec) - float(prior_stamp)) <= self.external_twist_timeout

    def get_effective_prior_source(self) -> str:
        if self.get_effective_control_mode() == 'manual':
            return self.manual_prior_source
        return self.prior_source

    def should_use_scan_deskew(self) -> bool:
        if not self.use_scan_deskew:
            return False
        if self.get_effective_control_mode() == 'manual' and self.manual_disable_scan_deskew:
            return False
        return True

    def get_external_body_twist(self, stamp_sec: Optional[float] = None) -> Tuple[float, float, float, str]:
        """Return body-frame velocity prior (vx, vy, wz, source)."""
        if stamp_sec is None:
            stamp_sec = self.now_sec()

        odom_fresh = self.is_prior_fresh(stamp_sec, self.prior_odom_stamp)
        imu_fresh = self.is_prior_fresh(stamp_sec, self.prior_imu_stamp)
        cmd_fresh = self.is_prior_fresh(stamp_sec, self.prior_cmd_stamp)

        src = self.get_effective_prior_source()
        if src == 'internal' or not (self.use_external_twist_prior or self.use_scan_deskew):
            return self.filtered_vx_body, self.filtered_vy_body, self.filtered_wz, 'internal'

        # Use wheel/cmd for translation. Prefer IMU yaw rate if available because yaw is critical
        # for fast scan matching and a 120-degree front scan has weak yaw observability.
        if src in ('wheel_imu', 'wheel'):
            if odom_fresh:
                wz = self.prior_imu_wz if (src == 'wheel_imu' and imu_fresh) else self.prior_odom_wz
                return self.prior_odom_vx, self.prior_odom_vy, wz, 'wheel_imu' if (src == 'wheel_imu' and imu_fresh) else 'wheel'
            return self.filtered_vx_body, self.filtered_vy_body, self.filtered_wz, 'internal'

        if src == 'cmd_vel':
            if cmd_fresh:
                wz = self.prior_imu_wz if imu_fresh else self.prior_cmd_wz
                return self.prior_cmd_vx, self.prior_cmd_vy, wz, 'cmd_vel_imu' if imu_fresh else 'cmd_vel'
            return self.filtered_vx_body, self.filtered_vy_body, self.filtered_wz, 'internal'

        # auto: wheel + IMU first, then cmd_vel + IMU, then internal prediction.
        if odom_fresh:
            wz = self.prior_imu_wz if imu_fresh else self.prior_odom_wz
            return self.prior_odom_vx, self.prior_odom_vy, wz, 'wheel_imu' if imu_fresh else 'wheel'
        if cmd_fresh:
            wz = self.prior_imu_wz if imu_fresh else self.prior_cmd_wz
            return self.prior_cmd_vx, self.prior_cmd_vy, wz, 'cmd_vel_imu' if imu_fresh else 'cmd_vel'
        return self.filtered_vx_body, self.filtered_vy_body, self.filtered_wz, 'internal'

    def angle_in_any_window(self, angle_rad: float) -> bool:
        if not self.use_angle_filter or not self.angle_windows:
            return True
        a = normalize_angle(angle_rad)
        for lo, hi in self.angle_windows:
            lo_n = normalize_angle(lo)
            hi_n = normalize_angle(hi)
            if lo_n <= hi_n:
                if lo_n <= a <= hi_n:
                    return True
            else:
                # window crosses +/- pi
                if a >= lo_n or a <= hi_n:
                    return True
        return False

    def scan_to_points(self, msg: LaserScan) -> Tuple[Array2, Array2]:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.shape[0]
        if n == 0:
            return np.empty((0, 2), dtype=np.float64), np.empty((0,), dtype=np.float64)

        raw_indices = np.arange(n, dtype=np.float64)
        raw_angles = msg.angle_min + raw_indices * msg.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges >= self.min_range
        valid &= ranges <= self.max_range
        if self.use_angle_filter and self.angle_windows:
            angle_mask = np.array([self.angle_in_any_window(float(a)) for a in raw_angles], dtype=bool)
            valid &= angle_mask

        ranges = ranges[valid]
        raw_angles = raw_angles[valid]
        raw_indices = raw_indices[valid]
        if ranges.size == 0:
            return np.empty((0, 2), dtype=np.float64), np.empty((0,), dtype=np.float64)

        step = self.downsample_step
        ranges = ranges[::step]
        raw_angles = raw_angles[::step]
        raw_indices = raw_indices[::step]

        x_l = ranges * np.cos(raw_angles)
        y_l = ranges * np.sin(raw_angles)
        if self.invert_scan_x:
            x_l = -x_l
        if self.invert_scan_y:
            y_l = -y_l

        c = math.cos(self.scan_yaw_offset)
        s = math.sin(self.scan_yaw_offset)
        x_b = c * x_l - s * y_l + self.scan_x_offset
        y_b = s * x_l + c * y_l + self.scan_y_offset
        pts = np.stack([x_b, y_b], axis=1).astype(np.float64)

        if self.should_use_scan_deskew() and pts.shape[0] > 0:
            stamp_sec = self.msg_stamp_to_sec(msg.header.stamp)
            vx, vy, wz, src = self.get_external_body_twist(stamp_sec)
            self.last_deskew_source = src
            motion_mag = abs(vx) + abs(vy) + abs(wz)
            if motion_mag > 1e-6:
                time_inc = float(msg.time_increment)
                if time_inc <= 0.0 and float(msg.scan_time) > 0.0 and n > 1:
                    time_inc = float(msg.scan_time) / float(n - 1)
                if time_inc > 0.0:
                    if self.deskew_center_scan:
                        scan_time = float(msg.scan_time) if float(msg.scan_time) > 0.0 else time_inc * float(n - 1)
                        ref_time = 0.5 * scan_time
                    else:
                        ref_time = 0.0
                    beam_dt = raw_indices * time_inc - ref_time
                    if self.deskew_max_time > 0.0:
                        beam_dt = np.clip(beam_dt, -self.deskew_max_time, self.deskew_max_time)

                    corrected = np.empty_like(pts)
                    for i, p in enumerate(pts):
                        dt_i = float(beam_dt[i])
                        dx_i = vx * dt_i
                        dy_i = vy * dt_i
                        dyaw_i = wz * dt_i
                        # Convert the point measured at beam time into the scan reference time.
                        # robot_delta_to_env_transform(center->beam) maps reference points into beam frame;
                        # invert it to express this beam point at the reference time.
                        R_env, t_env = robot_delta_to_env_transform(dx_i, dy_i, dyaw_i)
                        corrected[i, :] = R_env.T @ (p - t_env)
                    pts = corrected
        else:
            self.last_deskew_source = 'off'

        base_angles = np.arctan2(pts[:, 1], pts[:, 0]).astype(np.float64)
        return pts, base_angles

    def nearest_neighbor(self, src: Array2, dst: Array2) -> Tuple[Optional[Array2], Optional[Array2], Optional[Array2]]:
        if src.shape[0] == 0 or dst.shape[0] == 0:
            return None, None, None

        # Chunking avoids a large temporary array if the lidar returns many beams.
        chunk_size = 512
        all_idx: List[Array2] = []
        all_dist: List[Array2] = []
        for start in range(0, src.shape[0], chunk_size):
            block = src[start:start + chunk_size]
            diff = block[:, None, :] - dst[None, :, :]
            d2 = np.sum(diff * diff, axis=2)
            idx = np.argmin(d2, axis=1)
            dist = np.sqrt(d2[np.arange(block.shape[0]), idx])
            all_idx.append(idx)
            all_dist.append(dist)
        nn_idx = np.concatenate(all_idx)
        nn_d = np.concatenate(all_dist)
        return src, dst[nn_idx], nn_d

    def compute_observability(self, src_corr_transformed: Array2, mean_err: float) -> Tuple[Tuple[float, float, float], float]:
        """Estimate SE(2) observability from linearized point Jacobians."""
        if src_corr_transformed.shape[0] < 3:
            return (0.2, 0.2, 0.2), 1e9

        H = np.zeros((3, 3), dtype=np.float64)
        for p in src_corr_transformed:
            x = float(p[0])
            y = float(p[1])
            J = np.array([[1.0, 0.0, -y], [0.0, 1.0, x]], dtype=np.float64)
            H += J.T @ J

        H += np.diag([1e-9, 1e-9, 1e-9])
        try:
            H_inv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            return (0.2, 0.2, 0.2), 1e9

        sigma_r2 = max(float(mean_err), 0.01) ** 2
        Cov = sigma_r2 * H_inv
        sigma_x = math.sqrt(max(float(Cov[0, 0]), 1e-12))
        sigma_y = math.sqrt(max(float(Cov[1, 1]), 1e-12))
        sigma_yaw = math.sqrt(max(float(Cov[2, 2]), 1e-12))

        eigvals = np.linalg.eigvalsh(H)
        cond = float(eigvals[-1] / max(eigvals[0], 1e-12))
        return (sigma_x, sigma_y, sigma_yaw), cond

    def compute_scan_geometry_gate(self, base_angles: Array2) -> Tuple[float, float]:
        """
        Decrease trust in lateral/yaw when the visible scan sector is too narrow,
        front-heavy, or one-sided.
        """
        if base_angles.size < 10:
            return 0.2, 0.2

        dz = self.geometry_sector_deadzone
        left = np.count_nonzero(base_angles > dz)
        right = np.count_nonzero(base_angles < -dz)
        front = np.count_nonzero(np.abs(base_angles) <= dz)
        side_total = left + right

        side_balance = 0.0 if side_total == 0 else 2.0 * min(left, right) / float(side_total)
        ang_abs = np.abs(base_angles)
        spread = np.mean(ang_abs) / max(np.max(ang_abs), 1e-6)
        spread = clamp(float(spread), 0.0, 1.0)
        front_ratio = front / float(base_angles.size)
        front_penalty = clamp(1.0 - front_ratio, 0.0, 1.0)

        gate_y = clamp(0.20 + 0.45 * side_balance + 0.35 * spread, 0.0, 1.0)
        gate_y *= clamp(0.35 + 0.65 * front_penalty, 0.0, 1.0)
        gate_yaw = clamp(0.25 + 0.40 * side_balance + 0.35 * spread, 0.0, 1.0)
        gate_yaw *= clamp(0.45 + 0.55 * front_penalty, 0.0, 1.0)
        return gate_y, gate_yaw

    def sigma_to_alpha(self, sigma: float, sigma_ref: float, alpha_min: float, alpha_max: float) -> float:
        base = sigma_ref / max(float(sigma) + sigma_ref, 1e-12)
        base = clamp(base, 0.0, 1.0)
        return alpha_min + (alpha_max - alpha_min) * base

    def predicted_body_delta(self, dt: float, x_pred: Optional[Array2] = None, stamp_sec: Optional[float] = None) -> Tuple[float, float, float]:
        kf_delta = (self.filtered_vx_body * dt, self.filtered_vy_body * dt, self.filtered_wz * dt)
        if self.use_kalman_predict and self.kf.initialized and x_pred is not None:
            px = float(x_pred[0, 0])
            py = float(x_pred[1, 0])
            pyaw = normalize_angle(float(x_pred[2, 0]))
            dx_w = px - self.x
            dy_w = py - self.y
            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            dx_b = c * dx_w + s * dy_w
            dy_b = -s * dx_w + c * dy_w
            dyaw = normalize_angle(pyaw - self.yaw)
            kf_delta = (dx_b, dy_b, dyaw)

        if self.use_external_twist_prior:
            vx, vy, wz, src = self.get_external_body_twist(stamp_sec)
            self.last_used_prior_source = src
            self.last_used_prior_twist = (vx, vy, wz)
            if src != 'internal':
                ext_delta = (vx * dt, vy * dt, wz * dt)
                w = self.external_prior_weight
                return (
                    w * ext_delta[0] + (1.0 - w) * kf_delta[0],
                    w * ext_delta[1] + (1.0 - w) * kf_delta[1],
                    normalize_angle(w * ext_delta[2] + (1.0 - w) * kf_delta[2]),
                )
        else:
            self.last_used_prior_source = 'internal'
            self.last_used_prior_twist = (self.filtered_vx_body, self.filtered_vy_body, self.filtered_wz)

        return kf_delta

    def overwrite_prediction_with_delta(self, x_pred: Array2, init_delta: Tuple[float, float, float]) -> Array2:
        """When an external prior is used, make the Kalman prediction pose follow that prior too."""
        if self.last_used_prior_source == 'internal' or not self.kf.initialized:
            return x_pred
        dx_b, dy_b, dyaw = init_delta
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        dx_w = c * dx_b - s * dy_b
        dy_w = s * dx_b + c * dy_b
        out = x_pred.copy()
        out[0, 0] = self.x + dx_w
        out[1, 0] = self.y + dy_w
        out[2, 0] = normalize_angle(self.yaw + dyaw)
        vx_b, vy_b, wz = self.last_used_prior_twist
        out[3, 0] = c * vx_b - s * vy_b
        out[4, 0] = s * vx_b + c * vy_b
        out[5, 0] = wz
        return out

    def icp_relative_motion(
        self,
        prev_pts: Array2,
        curr_pts: Array2,
        init_delta_body: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float, float, int, Tuple[float, float, float], float]]:
        if prev_pts.shape[0] < self.min_correspondences or curr_pts.shape[0] < self.min_correspondences:
            return None

        R_total, t_total = robot_delta_to_env_transform(*init_delta_body)
        src = prev_pts.copy()
        dst = curr_pts.copy()

        mean_err = 1e9
        final_src_transformed: Optional[Array2] = None
        final_num_corr = 0

        for _ in range(self.icp_max_iterations):
            src_transformed = (R_total @ src.T).T + t_total
            matched_src, matched_dst, d = self.nearest_neighbor(src_transformed, dst)
            if matched_src is None or matched_dst is None or d is None:
                return None

            mask = d < self.icp_max_corr_dist
            if np.count_nonzero(mask) < self.min_correspondences:
                return None

            src_corr_all = matched_src[mask]
            dst_corr_all = matched_dst[mask]
            d_corr_all = d[mask]

            if self.icp_trim_ratio < 1.0 and src_corr_all.shape[0] > self.min_correspondences:
                keep_n = max(self.min_correspondences, int(src_corr_all.shape[0] * self.icp_trim_ratio))
                keep_idx = np.argpartition(d_corr_all, keep_n - 1)[:keep_n]
                src_corr = src_corr_all[keep_idx]
                dst_corr = dst_corr_all[keep_idx]
                d_corr = d_corr_all[keep_idx]
            else:
                src_corr = src_corr_all
                dst_corr = dst_corr_all
                d_corr = d_corr_all

            R_inc, t_inc = rigid_transform_2d(src_corr, dst_corr)
            if R_inc is None or t_inc is None:
                return None

            R_total = R_inc @ R_total
            t_total = R_inc @ t_total + t_inc
            mean_err = float(np.mean(d_corr))
            final_src_transformed = src_corr
            final_num_corr = int(src_corr.shape[0])

            inc_trans = float(np.linalg.norm(t_inc))
            inc_rot = abs(math.atan2(R_inc[1, 0], R_inc[0, 0]))
            if inc_trans < self.icp_converge_trans and inc_rot < self.icp_converge_rot:
                break

        if final_src_transformed is None:
            return None

        dx, dy, dyaw = env_transform_to_robot_delta(R_total, t_total)
        sigmas, cond = self.compute_observability(final_src_transformed, mean_err)
        return dx, dy, dyaw, mean_err, final_num_corr, sigmas, cond

    def dynamic_covariance(self) -> Tuple[float, float, float]:
        if self.publish_dynamic_covariance:
            sx, sy, syaw = self.last_sigmas
            ax, ay, ayaw = self.last_alphas
            cov_x = clamp(max(self.min_cov_x, sx * sx / max(ax, 1e-3)), self.min_cov_x, self.max_cov_x)
            cov_y = clamp(max(self.min_cov_y, sy * sy / max(ay, 1e-3)), self.min_cov_y, self.max_cov_y)
            cov_yaw = clamp(max(self.min_cov_yaw, syaw * syaw / max(ayaw, 1e-3)), self.min_cov_yaw, self.max_cov_yaw)
        else:
            cov_x, cov_y, cov_yaw = self.kalman_r_x, self.kalman_r_y, self.kalman_r_yaw
        return cov_x, cov_y, cov_yaw

    def publish_odom_and_tf(self, stamp, x: float, y: float, yaw: float, vx_body: float, vy_body: float, wz: float) -> None:
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

        cov_x, cov_y, cov_yaw = self.dynamic_covariance()
        odom.pose.covariance = [
            cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, cov_y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, cov_yaw,
        ]
        odom.twist.twist.linear.x = float(vx_body)
        odom.twist.twist.linear.y = float(vy_body)
        odom.twist.twist.angular.z = float(wz)
        odom.twist.covariance = [
            max(0.02, cov_x * 1.8), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, max(0.02, cov_y * 1.8), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, max(0.03, cov_yaw * 1.8),
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

    def publish_debug_msg(self, text: str) -> None:
        if self.debug_pub is not None:
            msg = String()
            msg.data = text
            self.debug_pub.publish(msg)

    def scan_callback(self, msg: LaserScan) -> None:
        self.callback_count += 1
        points, base_angles = self.scan_to_points(msg)
        if points.shape[0] < self.min_correspondences:
            self.publish_debug_msg(f'not_enough_points points={points.shape[0]}')
            return

        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if self.prev_points is None:
            self.prev_points = points
            self.prev_base_angles = base_angles
            self.last_stamp = stamp_sec
            self.last_pub_time = stamp_sec
            self.kf.initialize(self.x, self.y, self.yaw)
            self.publish_odom_and_tf(stamp, self.x, self.y, self.yaw, 0.0, 0.0, 0.0)
            self.publish_debug_msg('initialized')
            return

        if self.last_stamp is None:
            self.last_stamp = stamp_sec
            self.prev_points = points
            self.prev_base_angles = base_angles
            return

        dt = stamp_sec - self.last_stamp
        if dt <= 1e-6 or not math.isfinite(dt):
            self.prev_points = points
            self.prev_base_angles = base_angles
            self.last_stamp = stamp_sec
            return
        if dt > self.max_dt:
            # Too old: matching assumption breaks. Reset the reference scan.
            self.prev_points = points
            self.prev_base_angles = base_angles
            self.last_stamp = stamp_sec
            self.publish_debug_msg(f'reset_large_dt dt={dt:.3f}')
            return

        x_pred, p_pred = self.kf.predict_copy(dt)
        init_delta = self.predicted_body_delta(dt, x_pred=x_pred, stamp_sec=stamp_sec)
        x_pred = self.overwrite_prediction_with_delta(x_pred, init_delta)
        result = self.icp_relative_motion(self.prev_points, points, init_delta)

        self.prev_points = points
        self.prev_base_angles = base_angles
        self.last_stamp = stamp_sec

        if result is None:
            self.publish_debug_msg('icp_failed')
            return

        dx_body, dy_body, dyaw, mean_err, num_corr, sigmas, cond = result
        self.last_num_corr = num_corr
        self.last_match_err = mean_err
        self.last_sigmas = sigmas
        self.last_cond = cond

        if (not math.isfinite(mean_err)) or mean_err > self.reject_mean_err_above:
            self.publish_debug_msg(f'reject_err err={mean_err:.4f} corr={num_corr}')
            return

        if self.use_adaptive_constraint:
            sigma_x, sigma_y, sigma_yaw = sigmas
            alpha_x = self.sigma_to_alpha(sigma_x, self.sigma_ref_x, self.alpha_x_min, self.alpha_x_max)
            alpha_y = self.sigma_to_alpha(sigma_y, self.sigma_ref_y, self.alpha_y_min, self.alpha_y_max)
            alpha_yaw = self.sigma_to_alpha(sigma_yaw, self.sigma_ref_yaw, self.alpha_yaw_min, self.alpha_yaw_max)

            gate_y, gate_yaw = 1.0, 1.0
            if self.use_scan_geometry_gate:
                gate_y, gate_yaw = self.compute_scan_geometry_gate(base_angles)
                alpha_y *= clamp((1.0 - self.geometry_weight_y) + self.geometry_weight_y * gate_y, 0.0, 1.0)
                alpha_yaw *= clamp((1.0 - self.geometry_weight_yaw) + self.geometry_weight_yaw * gate_yaw, 0.0, 1.0)

            alpha_x = clamp(alpha_x, self.alpha_x_min, self.alpha_x_max)
            alpha_y = clamp(alpha_y, self.alpha_y_min, self.alpha_y_max)
            alpha_yaw = clamp(alpha_yaw, self.alpha_yaw_min, self.alpha_yaw_max)

            dx_body = alpha_x * dx_body + (1.0 - alpha_x) * init_delta[0]
            # For fast motion, weakly-observable y/yaw should fall back to the motion prior,
            # not to zero. This prevents a 120-degree front scan from dragging the pose sideways.
            dy_fallback = init_delta[1] if self.last_used_prior_source != 'internal' else self.dy_prior
            dyaw_fallback = init_delta[2] if self.last_used_prior_source != 'internal' else self.dyaw_prior
            dy_body = alpha_y * dy_body + (1.0 - alpha_y) * dy_fallback
            dyaw = alpha_yaw * dyaw + (1.0 - alpha_yaw) * dyaw_fallback
            self.last_alphas = (alpha_x, alpha_y, alpha_yaw)
            self.last_scan_gate = (gate_y, gate_yaw)
        else:
            self.last_alphas = (1.0, 1.0, 1.0)
            self.last_scan_gate = (1.0, 1.0)

        # Deadband and jump rejection.
        if abs(dx_body) < self.min_motion_trans:
            dx_body = 0.0
        if abs(dy_body) < self.min_motion_trans:
            dy_body = 0.0
        if abs(dyaw) < self.min_motion_rot:
            dyaw = 0.0

        trans_norm = math.hypot(dx_body, dy_body)
        if trans_norm < self.static_lock_trans and abs(dyaw) < self.static_lock_rot:
            dx_body = 0.0
            dy_body = 0.0
            dyaw = 0.0
            trans_norm = 0.0

        if trans_norm > self.reject_trans_above or abs(dyaw) > self.reject_rot_above:
            warn = f'Reject ICP jump: trans={trans_norm:.3f}, rot={dyaw:.3f}, err={mean_err:.3f}, corr={num_corr}, cond={cond:.1f}'
            self.get_logger().warn(warn)
            self.publish_debug_msg(warn)
            return

        # Body increment -> odom/world pose.
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        dx_world = cos_y * dx_body - sin_y * dy_body
        dy_world = sin_y * dx_body + cos_y * dy_body
        raw_x = self.x + dx_world
        raw_y = self.y + dy_world
        raw_yaw = normalize_angle(self.yaw + dyaw)

        cov_x, cov_y, cov_yaw = self.dynamic_covariance()
        if self.use_kalman_update:
            self.kf.commit_prediction(x_pred, p_pred)
            self.kf.update_pose(raw_x, raw_y, raw_yaw, [max(cov_x, self.kalman_r_x), max(cov_y, self.kalman_r_y), max(cov_yaw, self.kalman_r_yaw)])
            self.x, self.y, self.yaw = self.kf.get_pose()
        else:
            self.x, self.y, self.yaw = raw_x, raw_y, raw_yaw

        # Twist must be in child_frame_id/base_frame for nav_msgs/Odometry.
        raw_vx_body = dx_body / dt
        raw_vy_body = dy_body / dt
        raw_wz = dyaw / dt
        a = self.vel_alpha
        self.filtered_vx_body = (1.0 - a) * self.filtered_vx_body + a * raw_vx_body
        self.filtered_vy_body = (1.0 - a) * self.filtered_vy_body + a * raw_vy_body
        self.filtered_wz = (1.0 - a) * self.filtered_wz + a * raw_wz

        if self.use_kalman_update:
            # Feed the accepted scan-matching velocity back into the prediction model.
            # If an external prior was used, blend that prior more strongly because it is the
            # value that allowed ICP to remain in the correct basin during fast motion.
            if self.last_used_prior_source != 'internal':
                vx_b, vy_b, wz_b = self.last_used_prior_twist
                vx_world = math.cos(self.yaw) * vx_b - math.sin(self.yaw) * vy_b
                vy_world = math.sin(self.yaw) * vx_b + math.cos(self.yaw) * vy_b
                self.kf.blend_velocity(vx_world, vy_world, wz_b, max(self.velocity_feedback_alpha, 0.65))
            else:
                vx_world = math.cos(self.yaw) * raw_vx_body - math.sin(self.yaw) * raw_vy_body
                vy_world = math.sin(self.yaw) * raw_vx_body + math.cos(self.yaw) * raw_vy_body
                self.kf.blend_velocity(vx_world, vy_world, raw_wz, self.velocity_feedback_alpha)

        self.publish_odom_and_tf(stamp, self.x, self.y, self.yaw, self.filtered_vx_body, self.filtered_vy_body, self.filtered_wz)

        self.last_pub_x = self.x
        self.last_pub_y = self.y
        self.last_pub_yaw = self.yaw
        self.last_pub_time = stamp_sec

        if self.callback_count % self.debug_log_period == 0:
            ax, ay, ayaw = self.last_alphas
            gy, gyaw = self.last_scan_gate
            text = (
                f'ICP ok err={mean_err:.4f}, corr={num_corr}, cond={cond:.1f}, '
                f'sigma=({sigmas[0]:.3f},{sigmas[1]:.3f},{sigmas[2]:.3f}), '
                f'alpha=({ax:.2f},{ay:.2f},{ayaw:.2f}), gate=({gy:.2f},{gyaw:.2f}), '
                f'init=({init_delta[0]:.3f},{init_delta[1]:.3f},{init_delta[2]:.3f}), '
                f'prior={self.last_used_prior_source}, deskew={self.last_deskew_source}, '
                f'body=({dx_body:.3f},{dy_body:.3f},{dyaw:.3f}), '
                f'pose=({self.x:.3f},{self.y:.3f},{self.yaw:.3f})'
            )
            self.get_logger().info(text)
            self.publish_debug_msg(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanMatchOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
