#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import array
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan

class D435DepthToScan(Node):
    def __init__(self):
        super().__init__('d435_depth_to_scan')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_scan', '/d435_scan')
        self.declare_parameter('output_frame_id', 'camera_link')
        self.declare_parameter('scan_height_pixels', 40)
        self.declare_parameter('min_range', 0.25)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('range_percentile', 0.20)
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_scan = self.get_parameter('output_scan').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.scan_height_pixels = int(self.get_parameter('scan_height_pixels').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.range_percentile = float(self.get_parameter('range_percentile').value)
        self.fx = None
        self.cx = None
        self.pub = self.create_publisher(LaserScan, self.output_scan, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.cb_info, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)
        self.get_logger().info(f'd435_depth_to_scan: {self.depth_topic} -> {self.output_scan}')
    def cb_info(self, msg):
        self.fx = float(msg.k[0])
        self.cx = float(msg.k[2])
    def cb_depth(self, msg):
        if self.fx is None or self.cx is None:
            return
        if msg.encoding not in ['16UC1', 'mono16']:
            self.get_logger().warn_once(f'Unsupported depth encoding: {msg.encoding}')
            return
        data = array.array('H')
        data.frombytes(bytes(msg.data))
        w, h = msg.width, msg.height
        if len(data) < w * h:
            return
        row_center = h // 2
        half = max(1, self.scan_height_pixels // 2)
        row0, row1 = max(0, row_center - half), min(h - 1, row_center + half)
        angle_min = math.atan2((0 - self.cx), self.fx)
        angle_max = math.atan2(((w - 1) - self.cx), self.fx)
        angle_increment = (angle_max - angle_min) / max(1, w - 1)
        ranges = []
        for u in range(w):
            vals = []
            theta = math.atan2((u - self.cx), self.fx)
            c = math.cos(theta)
            if abs(c) < 1e-4:
                ranges.append(float('inf'))
                continue
            for v in range(row0, row1 + 1):
                z_mm = data[v * w + u]
                if z_mm == 0 or z_mm >= 65535:
                    continue
                z = z_mm / 1000.0
                r = z / c
                if self.min_range <= r <= self.max_range:
                    vals.append(r)
            if not vals:
                ranges.append(float('inf'))
            else:
                vals.sort()
                idx = int(max(0, min(len(vals)-1, self.range_percentile * (len(vals)-1))))
                ranges.append(vals[idx])
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.output_frame_id
        scan.angle_min = float(angle_min)
        scan.angle_max = float(angle_max)
        scan.angle_increment = float(angle_increment)
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = ranges
        self.pub.publish(scan)

def main():
    rclpy.init()
    node = D435DepthToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
