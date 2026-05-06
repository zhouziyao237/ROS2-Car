#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


def deg_360(angle_rad: float) -> float:
    return math.degrees(angle_rad) % 360.0


class FrontScanFilter(Node):
    """
    LaserScan sector filter.

    Input:
      /scan

    Output:
      /scan_front

    Default behavior:
      keep original 0~360 degree LaserScan geometry,
      but block 180~280 degree sector by setting ranges to inf.
    """

    def __init__(self):
        super().__init__("front_scan_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_front")

        self.declare_parameter("filter_mode", "block_sector")

        self.declare_parameter("block_start_deg", 180.0)
        self.declare_parameter("block_end_deg", 280.0)

        self.declare_parameter("min_range", 0.18)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("frame_id", "")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)

        self.filter_mode = str(self.get_parameter("filter_mode").value)

        self.block_start_deg = float(self.get_parameter("block_start_deg").value) % 360.0
        self.block_end_deg = float(self.get_parameter("block_end_deg").value) % 360.0

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(
            LaserScan,
            self.output_topic,
            pub_qos
        )

        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.on_scan,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f"front_scan_filter started: {self.input_topic} -> {self.output_topic}, "
            f"mode={self.filter_mode}, "
            f"block={self.block_start_deg:.1f}~{self.block_end_deg:.1f} deg, "
            f"range={self.min_range:.2f}~{self.max_range:.2f} m"
        )

    def angle_in_range(self, angle_deg: float, start_deg: float, end_deg: float) -> bool:
        angle_deg = angle_deg % 360.0
        start_deg = start_deg % 360.0
        end_deg = end_deg % 360.0

        if start_deg <= end_deg:
            return start_deg <= angle_deg <= end_deg

        return angle_deg >= start_deg or angle_deg <= end_deg

    def clean_range(self, value: float, out_range_max: float) -> float:
        if not math.isfinite(value):
            return math.inf

        if value < self.min_range:
            return math.inf

        if value > self.max_range:
            return math.inf

        if value > out_range_max:
            return math.inf

        return value

    def on_scan(self, msg: LaserScan):
        if len(msg.ranges) == 0 or msg.angle_increment == 0.0:
            return

        out = LaserScan()
        out.header = msg.header

        if self.frame_id:
            out.header.frame_id = self.frame_id

        # Keep original scan geometry.
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time

        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = min(msg.range_max, self.max_range) if msg.range_max > 0.0 else self.max_range

        has_intensity = len(msg.intensities) == len(msg.ranges)

        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if has_intensity else []

        for i in range(len(ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = deg_360(angle)

            if self.filter_mode == "block_sector" and self.angle_in_range(
                angle_deg,
                self.block_start_deg,
                self.block_end_deg
            ):
                ranges[i] = math.inf
            else:
                ranges[i] = self.clean_range(float(ranges[i]), out.range_max)

        out.ranges = ranges
        out.intensities = intensities

        self.pub.publish(out)


def main():
    rclpy.init()
    node = FrontScanFilter()

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