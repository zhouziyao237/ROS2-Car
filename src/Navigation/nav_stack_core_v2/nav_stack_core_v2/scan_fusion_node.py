#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan


class ScanFusionNode(Node):
    def __init__(self):
        super().__init__('scan_fusion_node')

        self.declare_parameter('lidar_scan_topic', '/scan_front')
        self.declare_parameter('depth_scan_topic', '/d435_scan')
        self.declare_parameter('output_scan_topic', '/scan_fused')

        self.declare_parameter('range_min', 0.18)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('depth_timeout_sec', 0.5)
        self.declare_parameter('enable_d435_clearing', False)

        self.lidar_scan_topic = self.get_parameter('lidar_scan_topic').value
        self.depth_scan_topic = self.get_parameter('depth_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value

        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.depth_timeout_sec = float(self.get_parameter('depth_timeout_sec').value)
        self.enable_d435_clearing = bool(self.get_parameter('enable_d435_clearing').value)

        self.last_depth_scan = None
        self.last_depth_time = None

        # 关键：匹配 /scan_front 的 BEST_EFFORT QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(
            LaserScan,
            self.output_scan_topic,
            pub_qos
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.lidar_scan_topic,
            self.on_lidar_scan,
            sensor_qos
        )

        self.depth_sub = self.create_subscription(
            LaserScan,
            self.depth_scan_topic,
            self.on_depth_scan,
            sensor_qos
        )

        self.get_logger().info(
            f'scan_fusion_node started: '
            f'{self.lidar_scan_topic} + {self.depth_scan_topic} -> {self.output_scan_topic}, '
            f'qos=BEST_EFFORT depth=1, '
            f'enable_d435_clearing={self.enable_d435_clearing}, '
            f'depth_timeout_sec={self.depth_timeout_sec}'
        )

    def valid_range(self, r):
        if not math.isfinite(r):
            return False
        if r < self.range_min:
            return False
        if r > self.range_max:
            return False
        return True

    def on_depth_scan(self, msg):
        self.last_depth_scan = msg
        self.last_depth_time = self.get_clock().now()

    def on_lidar_scan(self, lidar_msg):
        out = copy.deepcopy(lidar_msg)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = lidar_msg.header.frame_id

        if self.last_depth_scan is None or self.last_depth_time is None:
            self.pub.publish(out)
            return

        age = (self.get_clock().now() - self.last_depth_time).nanoseconds / 1e9
        if age > self.depth_timeout_sec:
            self.pub.publish(out)
            return

        depth = self.last_depth_scan

        if len(out.ranges) == 0 or len(depth.ranges) == 0:
            self.pub.publish(out)
            return

        # 简单角度匹配：把 depth_scan 的每个角度投到 lidar_scan 对应 index
        for j, dr in enumerate(depth.ranges):
            if not self.valid_range(dr):
                continue

            angle = depth.angle_min + j * depth.angle_increment

            if angle < out.angle_min or angle > out.angle_max:
                continue

            i = int(round((angle - out.angle_min) / out.angle_increment))
            if i < 0 or i >= len(out.ranges):
                continue

            lr = out.ranges[i]

            # marking：取更近的障碍物
            if (not self.valid_range(lr)) or dr < lr:
                out.ranges[i] = dr

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
