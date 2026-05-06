#!/usr/bin/env python3
import math
import bisect

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


def norm_2pi(a: float) -> float:
    a = a % (2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a


def deg_360(angle_rad: float) -> float:
    return math.degrees(norm_2pi(angle_rad))


class FrontScanFilter(Node):
    """
    /scan -> /scan_front

    当前雷达方向：
      0 deg   = 前方
      90 deg  = 左侧
      180 deg = 后方
      270 deg = 右侧

    ROS base_link：
      x = 前方
      y = 左侧

    所以：
      reverse_scan = False

    这个版本额外做：
      1. 屏蔽 180~280 度
      2. 固定输出 LaserScan 点数，避免 slam_toolbox / RF2O 输入结构跳变
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

        self.declare_parameter("reverse_scan", False)

        self.declare_parameter("force_fixed_count", True)
        self.declare_parameter("fixed_count", 1324)
        self.declare_parameter("fixed_angle_min", 0.0)
        self.declare_parameter("fixed_angle_max", 2.0 * math.pi)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.filter_mode = str(self.get_parameter("filter_mode").value)

        self.block_start_deg = float(self.get_parameter("block_start_deg").value) % 360.0
        self.block_end_deg = float(self.get_parameter("block_end_deg").value) % 360.0

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.reverse_scan = bool(self.get_parameter("reverse_scan").value)

        self.force_fixed_count = bool(self.get_parameter("force_fixed_count").value)
        self.fixed_count = int(self.get_parameter("fixed_count").value)
        self.fixed_angle_min = float(self.get_parameter("fixed_angle_min").value)
        self.fixed_angle_max = float(self.get_parameter("fixed_angle_max").value)

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, pub_qos)
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
            f"range={self.min_range:.2f}~{self.max_range:.2f} m, "
            f"reverse_scan={self.reverse_scan}, "
            f"force_fixed_count={self.force_fixed_count}, "
            f"fixed_count={self.fixed_count}"
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

    def build_angle_range_pairs(self, msg: LaserScan):
        pairs = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            a = norm_2pi(angle)
            rr = self.clean_range(float(r), self.max_range)
            pairs.append((a, rr))

        pairs.sort(key=lambda x: x[0])
        return pairs

    def nearest_range_at_angle(self, pairs, target_angle: float):
        if not pairs:
            return math.inf

        angles = [p[0] for p in pairs]
        target = norm_2pi(target_angle)

        idx = bisect.bisect_left(angles, target)

        candidates = []

        if idx < len(pairs):
            candidates.append(pairs[idx])

        if idx > 0:
            candidates.append(pairs[idx - 1])

        if idx == len(pairs):
            candidates.append(pairs[0])

        if idx == 0:
            candidates.append(pairs[-1])

        best_r = math.inf
        best_da = 999.0

        for a, r in candidates:
            da = abs(math.atan2(math.sin(a - target), math.cos(a - target)))
            if da < best_da:
                best_da = da
                best_r = r

        return best_r

    def make_fixed_scan(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header

        if self.frame_id:
            out.header.frame_id = self.frame_id

        out.angle_min = self.fixed_angle_min
        out.angle_max = self.fixed_angle_max
        out.angle_increment = (out.angle_max - out.angle_min) / float(self.fixed_count - 1)

        out.time_increment = 0.0
        out.scan_time = msg.scan_time

        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = min(msg.range_max, self.max_range) if msg.range_max > 0.0 else self.max_range

        pairs = self.build_angle_range_pairs(msg)

        ranges = []
        for i in range(self.fixed_count):
            angle = out.angle_min + i * out.angle_increment
            angle_deg = deg_360(angle)

            if self.filter_mode == "block_sector" and self.angle_in_range(
                angle_deg,
                self.block_start_deg,
                self.block_end_deg
            ):
                ranges.append(math.inf)
                continue

            ranges.append(self.nearest_range_at_angle(pairs, angle))

        out.ranges = ranges
        out.intensities = []

        return out

    def make_original_geometry_scan(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header

        if self.frame_id:
            out.header.frame_id = self.frame_id

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
        return out

    def on_scan(self, msg: LaserScan):
        if len(msg.ranges) == 0 or msg.angle_increment == 0.0:
            return

        if self.force_fixed_count:
            out = self.make_fixed_scan(msg)
        else:
            out = self.make_original_geometry_scan(msg)

        if self.reverse_scan:
            self.get_logger().warn_once(
                "reverse_scan=True is not recommended for this LiDAR."
            )

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