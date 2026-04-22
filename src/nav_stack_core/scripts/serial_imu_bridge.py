#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class SerialImuBridge(Node):
    def __init__(self):
        super().__init__("serial_imu_bridge")

        self.declare_parameter("input_topic", "/serial/gimbal_joint_state")
        self.declare_parameter("output_topic", "/imu/data_serial")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("assume_roll_zero", False)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.assume_roll_zero = bool(self.get_parameter("assume_roll_zero").value)

        self.pub = self.create_publisher(Imu, self.output_topic, 50)
        self.sub = self.create_subscription(Twist, self.input_topic, self.callback, 100)

        self.last_stamp = None
        self.last_pitch = None
        self.last_yaw = None

        self.get_logger().info(
            f"Bridge {self.input_topic} -> {self.output_topic} (frame={self.imu_frame_id})"
        )

    def callback(self, msg: Twist):
        now = self.get_clock().now()

        pitch = float(msg.angular.x)
        yaw = float(msg.angular.z)
        roll = 0.0 if self.assume_roll_zero else float(msg.angular.y)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_frame_id

        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        wy = 0.0
        wz = 0.0
        if self.last_stamp is not None and self.last_pitch is not None and self.last_yaw is not None:
            dt = (now - self.last_stamp).nanoseconds * 1e-9
            if dt > 1e-4:
                wy = (pitch - self.last_pitch) / dt
                wz = normalize_angle(yaw - self.last_yaw) / dt

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = wy
        imu.angular_velocity.z = wz

        imu.orientation_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.03,
        ]
        imu.angular_velocity_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 0.20, 0.0,
            0.0, 0.0, 0.10,
        ]
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.linear_acceleration_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 99999.0, 0.0,
            0.0, 0.0, 99999.0,
        ]

        self.pub.publish(imu)

        self.last_stamp = now
        self.last_pitch = pitch
        self.last_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = SerialImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
