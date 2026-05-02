#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('use_msg_child_frame', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.use_msg_child_frame = self.get_parameter('use_msg_child_frame').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            50
        )

        self.get_logger().info(
            f'Broadcasting TF from topic {self.odom_topic}: '
            f'{self.parent_frame} -> {self.child_frame}'
        )

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation

        valid = all(math.isfinite(v) for v in [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            q.x, q.y, q.z, q.w
        ])
        if not valid:
            self.get_logger().warn('Ignore odom with NaN/Inf pose.')
            return

        child = self.child_frame
        if self.use_msg_child_frame and msg.child_frame_id:
            child = msg.child_frame_id

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame if self.parent_frame else msg.header.frame_id
        t.child_frame_id = child

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = q

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()