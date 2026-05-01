#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class SearchState(Enum):
    IDLE = 0
    ROTATE_SCAN = 1
    FORWARD_SCAN = 2
    DONE = 3


class ShelfSearchManager(Node):
    def __init__(self):
        super().__init__('shelf_search_manager')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_search')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('rotate_speed', 0.35)
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('rotate_time_sec', 8.0)
        self.declare_parameter('forward_time_sec', 4.0)
        self.declare_parameter('front_safety_distance', 0.45)
        self.declare_parameter('front_angle_deg', 30.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.rotate_speed = float(self.get_parameter('rotate_speed').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.rotate_time_sec = float(self.get_parameter('rotate_time_sec').value)
        self.forward_time_sec = float(self.get_parameter('forward_time_sec').value)
        self.front_safety_distance = float(self.get_parameter('front_safety_distance').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

        self.state = SearchState.IDLE
        self.state_start_time = None
        self.front_clear = True

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.search_done_pub = self.create_publisher(Empty, '/search_done', 10)
        self.search_state_pub = self.create_publisher(String, '/shelf_search_state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/shelf_search_markers', 10)

        self.create_subscription(Empty, '/start_shelf_search', self.start_callback, 10)
        self.create_subscription(Empty, '/target_found', self.target_found_callback, 10)
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.loop)
        self.marker_timer = self.create_timer(0.5, self.publish_status_marker)

        self.get_logger().info('shelf_search_manager started.')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def start_callback(self, msg: Empty):
        if self.state not in [SearchState.IDLE, SearchState.DONE]:
            self.get_logger().warn('Search already running.')
            return

        self.state = SearchState.ROTATE_SCAN
        self.state_start_time = self.now_sec()
        self.get_logger().info('Shelf search started: ROTATE_SCAN.')

    def target_found_callback(self, msg: Empty):
        if self.state == SearchState.IDLE:
            self.get_logger().warn('Target found received, but search is not running.')
            return

        self.get_logger().info('Target found. Finish search.')
        self.finish_search()

    def scan_callback(self, scan: LaserScan):
        if len(scan.ranges) == 0:
            self.front_clear = True
            return

        front_angle = math.radians(self.front_angle_deg)
        valid_ranges = []

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if abs(angle) <= front_angle:
                if scan.range_min <= r <= scan.range_max:
                    valid_ranges.append(r)

        if len(valid_ranges) == 0:
            self.front_clear = True
            return

        min_front = min(valid_ranges)
        self.front_clear = min_front > self.front_safety_distance

    def loop(self):
        state_msg = String()
        state_msg.data = self.state.name
        self.search_state_pub.publish(state_msg)

        if self.state in [SearchState.IDLE, SearchState.DONE]:
            return

        elapsed = self.now_sec() - self.state_start_time

        if self.state == SearchState.ROTATE_SCAN:
            cmd = Twist()
            cmd.angular.z = self.rotate_speed
            self.cmd_pub.publish(cmd)

            if elapsed >= self.rotate_time_sec:
                self.stop_robot()
                self.state = SearchState.FORWARD_SCAN
                self.state_start_time = self.now_sec()
                self.get_logger().info('Switch to FORWARD_SCAN.')

        elif self.state == SearchState.FORWARD_SCAN:
            if not self.front_clear:
                self.get_logger().warn('Front obstacle too close. Finish search.')
                self.finish_search()
                return

            cmd = Twist()
            cmd.linear.x = self.forward_speed
            self.cmd_pub.publish(cmd)

            if elapsed >= self.forward_time_sec:
                self.finish_search()

    def stop_robot(self):
        cmd = Twist()
        if not rclpy.ok():
            return

        try:
            self.cmd_pub.publish(cmd)
        except Exception:
            # ROS context may already be shutting down during Ctrl+C teardown.
            pass

    def finish_search(self):
        self.stop_robot()
        self.state = SearchState.DONE
        self.search_done_pub.publish(Empty())
        self.get_logger().info('Search finished. Publish /search_done.')

    def publish_status_marker(self):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'shelf_search_status'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.2
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.text = f'Shelf Search: {self.state.name}\nFront clear: {self.front_clear}'

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ShelfSearchManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
