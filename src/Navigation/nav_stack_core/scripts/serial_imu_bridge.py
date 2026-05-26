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
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("assume_roll_zero", False)
        self.declare_parameter("roll_scale", 1.0)
        self.declare_parameter("pitch_scale", 1.0)
        self.declare_parameter("yaw_scale", 1.0)
        self.declare_parameter("yaw_offset_deg", 0.0)
        self.declare_parameter("pitch_deadband_deg", 0.20)
        self.declare_parameter("yaw_deadband_deg", 0.25)
        self.declare_parameter("angular_velocity_deadband_y", 0.03)
        self.declare_parameter("angular_velocity_deadband_z", 0.03)
        self.declare_parameter("max_angular_velocity_y", 3.0)
        self.declare_parameter("max_angular_velocity_z", 3.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.assume_roll_zero = bool(self.get_parameter("assume_roll_zero").value)
        self.roll_scale = float(self.get_parameter("roll_scale").value)
        self.pitch_scale = float(self.get_parameter("pitch_scale").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.yaw_offset = math.radians(float(self.get_parameter("yaw_offset_deg").value))
        self.pitch_deadband = math.radians(float(self.get_parameter("pitch_deadband_deg").value))
        self.yaw_deadband = math.radians(float(self.get_parameter("yaw_deadband_deg").value))
        self.angular_velocity_deadband_y = float(
            self.get_parameter("angular_velocity_deadband_y").value
        )
        self.angular_velocity_deadband_z = float(
            self.get_parameter("angular_velocity_deadband_z").value
        )
        self.max_angular_velocity_y = max(
            0.0, float(self.get_parameter("max_angular_velocity_y").value)
        )
        self.max_angular_velocity_z = max(
            0.0, float(self.get_parameter("max_angular_velocity_z").value)
        )

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

        # Serial IMU convention provided by the robot:
        #   yaw   : left turn is positive
        #   roll  : upward tilt is positive
        #   pitch : right tilt is positive
        #
        # Convert that convention into ROS REP-103 roll/pitch/yaw:
        #   ros_yaw   = imu_yaw
        #   ros_pitch = imu_roll
        #   ros_roll  = -imu_pitch
        serial_pitch = float(msg.angular.x)
        raw_yaw = float(msg.angular.z)
        serial_roll = 0.0 if self.assume_roll_zero else float(msg.angular.y)

        raw_pitch = serial_roll
        raw_roll = -serial_pitch

        pitch = self.pitch_scale * raw_pitch
        yaw = normalize_angle(self.yaw_scale * raw_yaw + self.yaw_offset)
        roll = self.roll_scale * raw_roll

        if self.last_pitch is not None:
            if abs(pitch - self.last_pitch) < self.pitch_deadband:
                pitch = self.last_pitch
        if self.last_yaw is not None:
            if abs(normalize_angle(yaw - self.last_yaw)) < self.yaw_deadband:
                yaw = self.last_yaw

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

        if abs(wy) < self.angular_velocity_deadband_y:
            wy = 0.0
        if abs(wz) < self.angular_velocity_deadband_z:
            wz = 0.0
        if self.max_angular_velocity_y > 0.0:
            wy = max(-self.max_angular_velocity_y, min(self.max_angular_velocity_y, wy))
        if self.max_angular_velocity_z > 0.0:
            wz = max(-self.max_angular_velocity_z, min(self.max_angular_velocity_z, wz))

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
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
