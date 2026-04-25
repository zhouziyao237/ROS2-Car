#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class FrontScanFilter(Node):
    """
    从原始 /scan 中裁出前向扇区，并且：
    1. 输出固定点数
    2. 保留原始扫描方向（不再粗暴 abs(angle_increment)）
    3. 发布 QoS 改成 RELIABLE，兼容 RF2O / 部分 SLAM 节点
    """

    def __init__(self):
        super().__init__("front_scan_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_front")
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("center_angle_deg", 0.0)
        self.declare_parameter("min_range", 0.18)
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("frame_id", "")

        # 新增：锁定输出几何，避免每帧点数变化
        self.declare_parameter("lock_geometry", True)
        self.declare_parameter("fixed_count", 0)  # 0 表示首次收到 scan 后自动锁定
        self.declare_parameter("force_positive_increment", False)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.fov_rad = math.radians(float(self.get_parameter("fov_deg").value))
        self.center_rad = math.radians(float(self.get_parameter("center_angle_deg").value))
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.lock_geometry = bool(self.get_parameter("lock_geometry").value)
        self.fixed_count = int(self.get_parameter("fixed_count").value)
        self.force_positive_increment = bool(self.get_parameter("force_positive_increment").value)

        # 订阅原始 scan：继续用 sensor data 风格
        sub_qos = qos_profile_sensor_data

        # 发布 scan_front：改成 RELIABLE，避免和 RF2O / slam_toolbox QoS 不兼容
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(LaserScan, self.output_topic, pub_qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.on_scan, sub_qos)
        # 锁定后的输出几何
        self._geometry_locked = False
        self._locked_step = None          # abs(angle_increment)
        self._locked_count = None
        self._locked_sign = None          # +1 or -1

        self.get_logger().info(
            f"Filtering {self.input_topic} -> {self.output_topic}, "
            f"fov={math.degrees(self.fov_rad):.1f}deg "
            f"center={math.degrees(self.center_rad):.1f}deg, "
            f"lock_geometry={self.lock_geometry}, fixed_count={self.fixed_count}"
        )

    def _lock_geometry_if_needed(self, msg: LaserScan):
        if self._geometry_locked:
            return

        raw_inc = float(msg.angle_increment)
        step = abs(raw_inc)
        if step <= 0.0:
            return

        sign = 1 if raw_inc >= 0.0 else -1
        if self.force_positive_increment:
            sign = 1

        if self.fixed_count > 1:
            count = self.fixed_count
        else:
            count = max(2, int(round(self.fov_rad / step)) + 1)

        self._locked_step = step
        self._locked_count = count
        self._locked_sign = sign
        self._geometry_locked = True

        self.get_logger().info(
            f"Locked scan_front geometry: count={count}, "
            f"step={step:.8f} rad ({math.degrees(step):.4f} deg), sign={sign}"
        )

    def find_raw_index(self, msg: LaserScan, angle: float) -> Optional[int]:
        if msg.angle_increment == 0.0 or len(msg.ranges) == 0:
            return None

        best: Optional[Tuple[float, int]] = None

        # 允许少量 2π wrap 搜索
        for turns in range(-2, 3):
            candidate = angle + turns * 2.0 * math.pi
            idx = int(round((candidate - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                raw_angle = msg.angle_min + idx * msg.angle_increment
                err = abs(normalize_angle(raw_angle - angle))
                if best is None or err < best[0]:
                    best = (err, idx)

        return None if best is None else best[1]

    def clean_range(self, value: float, out_range_max: float) -> float:
        if not math.isfinite(value):
            return math.inf
        if value < self.min_range or value > self.max_range:
            return math.inf
        if value > out_range_max:
            return math.inf
        return value

    def on_scan(self, msg: LaserScan):
        raw_inc = float(msg.angle_increment)
        if raw_inc == 0.0 or len(msg.ranges) == 0:
            return

        if self.lock_geometry:
            self._lock_geometry_if_needed(msg)
            if not self._geometry_locked:
                return
            step = self._locked_step
            count = self._locked_count
            sign = self._locked_sign
        else:
            step = abs(raw_inc)
            if step <= 0.0:
                return
            sign = 1 if raw_inc >= 0.0 else -1
            if self.force_positive_increment:
                sign = 1
            count = self.fixed_count if self.fixed_count > 1 else max(2, int(round(self.fov_rad / step)) + 1)

        out_inc = sign * step

        # 保留扫描方向：
        # sign > 0: 从 center - fov/2 到 center + fov/2
        # sign < 0: 从 center + fov/2 到 center - fov/2
        if sign > 0:
            angle_min = self.center_rad - self.fov_rad * 0.5
        else:
            angle_min = self.center_rad + self.fov_rad * 0.5

        angle_max = angle_min + (count - 1) * out_inc

        out = LaserScan()
        out.header = msg.header
        if self.frame_id:
            out.header.frame_id = self.frame_id

        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = out_inc
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = min(msg.range_max, self.max_range) if msg.range_max > 0.0 else self.max_range

        has_intensity = len(msg.intensities) == len(msg.ranges)

        ranges = [math.inf] * count
        intensities = [0.0] * count

        for i in range(count):
            angle = angle_min + i * out_inc
            idx = self.find_raw_index(msg, angle)
            if idx is None:
                continue

            ranges[i] = self.clean_range(float(msg.ranges[idx]), out.range_max)
            if has_intensity:
                intensities[i] = float(msg.intensities[idx])

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
