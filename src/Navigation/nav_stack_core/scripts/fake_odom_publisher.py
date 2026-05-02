#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('rate', 20.0)

        # 演示用：给一个恒定速度
        self.declare_parameter('linear_x', 0.03)   # m/s
        self.declare_parameter('linear_y', 0.0)    # m/s
        self.declare_parameter('angular_z', 0.08)  # rad/s

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.rate = float(self.get_parameter('rate').value)

        self.vx = float(self.get_parameter('linear_x').value)
        self.vy = float(self.get_parameter('linear_y').value)
        self.wz = float(self.get_parameter('angular_z').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_pub = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info(
            f'Fake odom started: vx={self.vx}, vy={self.vy}, wz={self.wz}'
        )

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or not math.isfinite(dt):
            return

        # 在机器人局部速度下积分
        dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        dyaw = self.wz * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # 发布 odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        # 发布 TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_pub.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()