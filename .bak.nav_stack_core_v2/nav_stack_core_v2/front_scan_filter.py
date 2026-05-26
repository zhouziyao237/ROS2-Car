#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

    目标：
      1. RF2O 继续吃原始 /scan
      2. slam_toolbox 吃 /scan_front
      3. /scan_front 保留原始 /scan 的时间戳，避免 SLAM 拿错时间的 TF
      4. /scan_front 低延迟输出，不堆积旧 scan
      5. 固定 LaserScan 点数，避免 scan 结构跳变
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

        # 程序内限频：给 slam_toolbox 用 5Hz 左右，避免 12Hz 堆积
        self.declare_parameter("output_max_hz", 5.0)

        # 时间戳策略：
        # input = 保留原始 /scan 时间戳，推荐用于 SLAM 时间同步
        # now   = 使用当前时间，只用于特殊调试
        self.declare_parameter("stamp_policy", "now")

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

        self.output_max_hz = float(self.get_parameter("output_max_hz").value)
        self.stamp_policy = str(self.get_parameter("stamp_policy").value).lower().strip()

        self.last_pub_time = None

        # 关键：低延迟，不堆积旧 scan
        # slam_toolbox 和 rviz 对 /scan_front 订阅常见是 BEST_EFFORT
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, pub_qos)

        # 输入 /scan 使用 sensor_data QoS，适合雷达
        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"front_scan_filter started: {self.input_topic} -> {self.output_topic}, "
            f"mode={self.filter_mode}, "
            f"block={self.block_start_deg:.1f}~{self.block_end_deg:.1f} deg, "
            f"range={self.min_range:.2f}~{self.max_range:.2f} m, "
            f"reverse_scan={self.reverse_scan}, "
            f"force_fixed_count={self.force_fixed_count}, "
            f"fixed_count={self.fixed_count}, "
            f"output_max_hz={self.output_max_hz}, "
            f"stamp_policy={self.stamp_policy}, "
            f"pub_qos=BEST_EFFORT depth=10"
        )

    def should_publish_now(self) -> bool:
        if self.output_max_hz <= 0.0:
            return True

        now = self.get_clock().now()

        if self.last_pub_time is None:
            self.last_pub_time = now
            return True

        dt = (now - self.last_pub_time).nanoseconds / 1e9
        if dt >= (1.0 / self.output_max_hz):
            self.last_pub_time = now
            return True

        return False

    def stamp_output(self, out: LaserScan, msg: LaserScan):
        if self.stamp_policy == "now":
            out.header.stamp = self.get_clock().now().to_msg()
        else:
            out.header.stamp = msg.header.stamp

        if self.frame_id:
            out.header.frame_id = self.frame_id

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
        self.stamp_output(out, msg)

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
        self.stamp_output(out, msg)

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

        # 关键：限频在程序内做，旧 scan 直接丢，不排队
        if not self.should_publish_now():
            return

        if self.force_fixed_count:
            out = self.make_fixed_scan(msg)
        else:
            out = self.make_original_geometry_scan(msg)

        if self.reverse_scan:
            self.get_logger().warn(
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
