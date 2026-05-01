#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray


class SimTagZonePublisher(Node):
    def __init__(self):
        super().__init__('sim_tag_zone_publisher')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('trigger_distance', 0.55)
        self.declare_parameter('republish_sec', 3.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.trigger_distance = float(self.get_parameter('trigger_distance').value)
        self.republish_sec = float(self.get_parameter('republish_sec').value)

        self.tags = {
            1: (-1.2, -2.4),
            2: (1.2, -2.4),
        }
        self.last_pub_time = {tag_id: -999.0 for tag_id in self.tags.keys()}

        self.tag_pub = self.create_publisher(Int32, '/tag_zone_id', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sim_tag_markers', 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(0.5, self.publish_markers)

        self.get_logger().info('sim_tag_zone_publisher started.')
        self.get_logger().info('When robot is close to virtual floor tag, publish /tag_zone_id.')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, odom: Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        now = self.now_sec()

        for tag_id, (tx, ty) in self.tags.items():
            dist = math.hypot(x - tx, y - ty)
            if dist <= self.trigger_distance and now - self.last_pub_time[tag_id] >= self.republish_sec:
                msg = Int32()
                msg.data = tag_id
                self.tag_pub.publish(msg)
                self.last_pub_time[tag_id] = now
                self.get_logger().info(f'Publish virtual tag_zone_id={tag_id}, distance={dist:.2f}')

    def publish_markers(self):
        arr = MarkerArray()
        for tag_id, (x, y) in self.tags.items():
            cube = Marker()
            cube.header.frame_id = 'map'
            cube.header.stamp = self.get_clock().now().to_msg()
            cube.ns = 'sim_tags'
            cube.id = tag_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = 0.03
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.5
            cube.scale.y = 0.5
            cube.scale.z = 0.03
            cube.color.r = 0.0 if tag_id == 1 else 0.0
            cube.color.g = 1.0 if tag_id == 1 else 0.0
            cube.color.b = 0.0 if tag_id == 1 else 1.0
            cube.color.a = 0.8
            arr.markers.append(cube)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'sim_tag_text'
            text.id = 100 + tag_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.45
            text.pose.orientation.w = 1.0
            text.scale.z = 0.32
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'TAG {tag_id}'
            arr.markers.append(text)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SimTagZonePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
