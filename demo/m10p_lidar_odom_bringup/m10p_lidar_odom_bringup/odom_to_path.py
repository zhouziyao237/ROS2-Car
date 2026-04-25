#!/usr/bin/env python3
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('odom_topic', '/lidar/odom')
        self.declare_parameter('path_topic', '/lidar/path')
        self.declare_parameter('max_points', 3000)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.max_points = int(self.get_parameter('max_points').value)

        self.poses = deque(maxlen=self.max_points)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, 20)
        self.get_logger().info(f'Converting {self.odom_topic} -> {self.path_topic}')

    def cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.poses.append(ps)

        path = Path()
        path.header = msg.header
        path.poses = list(self.poses)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
