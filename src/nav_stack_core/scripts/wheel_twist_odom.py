#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TwistWithCovarianceStamped


def yaw_to_quat(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class WheelTwistOdom(Node):
    def __init__(self):
        super().__init__("wheel_twist_odom")

        self.declare_parameter("input_topic", "/wheel/twist")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame", "wheel_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(
            TwistWithCovarianceStamped, self.input_topic, self.callback, 50
        )

        self.get_logger().info(
            f"Integrating {self.input_topic} -> {self.odom_topic}, TF {self.odom_frame}->{self.base_frame}"
        )

    def callback(self, msg: TwistWithCovarianceStamped):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 1e-4:
            return
        self.last_time = now

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += (vx * cos_y - vy * sin_y) * dt
        self.y += (vx * sin_y + vy * cos_y) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

        qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist = msg.twist

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


def main(args=None):
    rclpy.init(args=args)
    node = WheelTwistOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
