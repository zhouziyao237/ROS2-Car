#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Rf2oOdomAdapter(Node):
    def __init__(self):
        super().__init__("rf2o_odom_adapter")

        self.declare_parameter("input_odom_topic", "/odom_laser")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_odom_topic", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("zero_z", True)

        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_odom_topic = bool(self.get_parameter("publish_odom_topic").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.zero_z = bool(self.get_parameter("zero_z").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, qos)
        self.tf_pub = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.on_odom,
            qos,
        )

        self.get_logger().info(
            f"rf2o_odom_adapter started: {self.input_odom_topic} -> {self.output_odom_topic}, "
            f"TF {self.odom_frame}->{self.base_frame}, stamp_policy=now, qos depth=1"
        )

    def on_odom(self, msg: Odometry):
        stamp = self.get_clock().now().to_msg()

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist

        if self.zero_z:
            out.pose.pose.position.z = 0.0

        if self.publish_odom_topic:
            self.odom_pub.publish(out)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = out.pose.pose.position.x
            tf_msg.transform.translation.y = out.pose.pose.position.y
            tf_msg.transform.translation.z = 0.0 if self.zero_z else out.pose.pose.position.z
            tf_msg.transform.rotation = out.pose.pose.orientation

            self.tf_pub.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Rf2oOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()