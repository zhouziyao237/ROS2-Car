#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WheelTwistOdom(Node):
    def __init__(self):
        super().__init__("wheel_twist_odom")

        self.declare_parameter("input_topic", "/wheel/twist")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("use_msg_stamp", True)
        self.declare_parameter("max_dt", 0.20)
        self.declare_parameter("linear_deadband_x", 0.005)
        self.declare_parameter("linear_deadband_y", 0.005)
        self.declare_parameter("angular_deadband_z", 0.010)
        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("use_cmd_vel_input", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_selected")
        self.declare_parameter("cmd_cov_vx", 0.50)
        self.declare_parameter("cmd_cov_vy", 0.50)
        self.declare_parameter("cmd_cov_wz", 0.80)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_msg_stamp = bool(self.get_parameter("use_msg_stamp").value)
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.linear_deadband_x = float(self.get_parameter("linear_deadband_x").value)
        self.linear_deadband_y = float(self.get_parameter("linear_deadband_y").value)
        self.angular_deadband_z = float(self.get_parameter("angular_deadband_z").value)
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.use_cmd_vel_input = bool(self.get_parameter("use_cmd_vel_input").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_cov_vx = float(self.get_parameter("cmd_cov_vx").value)
        self.cmd_cov_vy = float(self.get_parameter("cmd_cov_vy").value)
        self.cmd_cov_wz = float(self.get_parameter("cmd_cov_wz").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None
        self.last_imu_yaw = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        if self.use_cmd_vel_input:
            self.sub = self.create_subscription(
                Twist, self.cmd_vel_topic, self.cmd_callback, 50
            )
            input_desc = self.cmd_vel_topic
            input_mode = "cmd_vel"
        else:
            self.sub = self.create_subscription(
                TwistWithCovarianceStamped, self.input_topic, self.twist_callback, 50
            )
            input_desc = self.input_topic
            input_mode = "wheel_feedback"
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 50
        ) if self.use_imu_yaw and self.imu_topic else None

        self.get_logger().info(
            f"Integrating {input_desc} -> {self.odom_topic}, "
            f"mode={input_mode}, publish_tf={self.publish_tf}, use_imu_yaw={self.use_imu_yaw}"
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm < 1e-12:
            return
        self.last_imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def get_stamp(self, msg: TwistWithCovarianceStamped) -> Time:
        if self.use_msg_stamp:
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds > 0:
                return stamp
        return self.get_clock().now()

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        return 0.0 if abs(value) < deadband else value

    def default_cmd_covariance(self):
        return [
            self.cmd_cov_vx, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.cmd_cov_vy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.cmd_cov_wz,
        ]

    def publish_odom(self, stamp: Time, vx: float, vy: float, wz: float, covariance):
        qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = covariance

        # Keep pose covariance conservative; twist covariance comes from wheel feedback.
        odom.pose.covariance = [
            0.08, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.08, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.12,
        ]
        self.odom_pub.publish(odom)

        if self.tf_pub is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.sendTransform(t)

    def integrate_sample(self, now: Time, vx: float, vy: float, wz: float, covariance):
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt < 0.0:
            self.get_logger().warn("Received non-monotonic wheel timestamp, resetting integrator clock")
            self.last_time = now
            return
        if dt <= 1e-4:
            return
        self.last_time = now

        if self.use_imu_yaw and self.last_imu_yaw is not None:
            self.yaw = self.last_imu_yaw

        if self.max_dt > 0.0 and dt > self.max_dt:
            self.get_logger().warn(
                f"Skipping wheel pose integration for large dt={dt:.3f}s (max_dt={self.max_dt:.3f}s)"
            )
            self.publish_odom(now, vx, vy, wz, covariance)
            return

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += (vx * cos_y - vy * sin_y) * dt
        self.y += (vx * sin_y + vy * cos_y) * dt
        if self.use_imu_yaw and self.last_imu_yaw is not None:
            self.yaw = self.last_imu_yaw
        else:
            self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

        self.publish_odom(now, vx, vy, wz, covariance)

    def twist_callback(self, msg: TwistWithCovarianceStamped):
        now = self.get_stamp(msg)
        vx = self.apply_deadband(float(msg.twist.twist.linear.x), self.linear_deadband_x)
        vy = self.apply_deadband(float(msg.twist.twist.linear.y), self.linear_deadband_y)
        wz = self.apply_deadband(float(msg.twist.twist.angular.z), self.angular_deadband_z)
        covariance = list(msg.twist.covariance)
        self.integrate_sample(now, vx, vy, wz, covariance)

    def cmd_callback(self, msg: Twist):
        now = self.get_clock().now()
        vx = self.apply_deadband(float(msg.linear.x), self.linear_deadband_x)
        vy = self.apply_deadband(float(msg.linear.y), self.linear_deadband_y)
        wz = self.apply_deadband(float(msg.angular.z), self.angular_deadband_z)
        covariance = self.default_cmd_covariance()
        self.integrate_sample(now, vx, vy, wz, covariance)


def main(args=None):
    rclpy.init(args=args)
    node = WheelTwistOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
