#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SimModelStateOdomNode(Node):
    def __init__(self):
        super().__init__('sim_model_state_odom_node')

        self.declare_parameter('robot_name', 'logistics_robot')
        self.declare_parameter('model_states_topic', '/model_states')
        self.declare_parameter('odom_topic', '/odom_model_debug')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)

        self.robot_name = self.get_parameter('robot_name').value
        self.model_states_topic = self.get_parameter('model_states_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 30)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.cb,
            30
        )

        self.get_logger().info(
            f'sim_model_state_odom_node started: source={self.model_states_topic}, robot={self.robot_name}'
        )

    def cb(self, msg: ModelStates):
        if self.robot_name not in msg.name:
            self.get_logger().warn(f'robot {self.robot_name} not in model_states')
            return

        i = msg.name.index(self.robot_name)
        pose = msg.pose[i]
        twist = msg.twist[i] if i < len(msg.twist) else None

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose

        if twist is not None:
            odom.twist.twist = twist

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SimModelStateOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
