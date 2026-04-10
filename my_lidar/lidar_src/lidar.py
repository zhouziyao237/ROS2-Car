#!/usr/bin/env python3
import sys
import os
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ==================== Import sdk_py ====================
from ament_index_python.packages import get_package_prefix

try:
    pkg_path = get_package_prefix('my_lidar')
    sdk_dir = os.path.join(pkg_path, '../../lidar_src/sdk_lib')
    print("[INFO] PKG PATH: " + pkg_path)
    if sdk_dir not in sys.path:
        sys.path.append(sdk_dir)
    import sdk_py
except Exception as e:
    raise ImportError(f"Unable to import sdk_py, please check package installation. Error: {e}")

print("SDK version:", sdk_py.get_version() if hasattr(sdk_py, 'get_version') else "unknown")

# 检查是否支持高级 API（回调方式）
HAS_ADVANCED_API = all(hasattr(sdk_py, name) for name in [
    'setSDKCallBackFunErrorCode', 'setSDKCallBackFunSecondInfo',
    'setSDKCallBackFunPointCloud', 'hcSDKInitialize', 'hcSDKUnInit'
])

if HAS_ADVANCED_API:
    print("Using advanced SDK API (callback mode)")
else:
    print("Advanced SDK API not available, using Dev class (polling mode)")

# ==================== LidarDriver (Encapsulation of Dev) ====================
class LidarDriver:
    """Encapsulation of sdk_py.Dev for polling mode"""
    def __init__(self):
        self.dev = None
        self.initialized = False

    def initialize(self, port, baudrate=115200, reconn=True):
        try:
            self.dev = sdk_py.Dev()
            result = self.dev.Initialize(port, baudrate, reconn)
            if result != 0:
                print(f"Device initialization failed, error code: {result}")
                return False
            self.initialized = True
            return True
        except Exception as e:
            print(f"Initialization exception: {e}")
            return False

    def get_scan_data(self, reverse=True):
        if not self.initialized or self.dev is None:
            return []
        try:
            data_list = self.dev.GetScanData(reverse)
            return data_list if data_list else []
        except Exception as e:
            print(f"Data acquisition exception: {e}")
            return []

    def get_last_error(self):
        if self.dev:
            try:
                return self.dev.GetLastErrCode()
            except:
                pass
        return -1

    def shutdown(self):
        if self.dev:
            try:
                self.dev.Uninit()
            except:
                pass
        self.initialized = False

# ==================== ROS2 Node ====================
class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('lidar_model', 'X2BF')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_frequency', 6.0)
        self.declare_parameter('range_min', 0.06)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('reverse', True)
        self.declare_parameter('debug_log', False)  # 日志开关，默认关闭

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        model = self.get_parameter('lidar_model').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_freq = self.get_parameter('scan_frequency').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.reverse = self.get_parameter('reverse').value
        self.debug_log = self.get_parameter('debug_log').value  # 获取开关值

        self.get_logger().info(f"Requested lidar model: {model}")

        # 创建发布者（默认 QoS，兼容 RViz2）
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        if HAS_ADVANCED_API:
            self._init_advanced_mode(port, baudrate, model)
        else:
            self._init_polling_mode(port, baudrate)

    def _init_advanced_mode(self, port, baudrate, model):
        """使用高级 API 初始化（回调方式）"""
        try:
            from functools import partial
            sdk_py.setSDKCallBackFunErrorCode(partial(self._error_callback))
            sdk_py.setSDKCallBackFunSecondInfo(partial(self._second_info_callback))
            sdk_py.setSDKCallBackFunPointCloud(partial(self._point_cloud_callback))

            ret = sdk_py.hcSDKInitialize(port, model, baudrate, 2, False, False, False)
            if ret != 1:
                self.get_logger().error(f"SDK initialization failed, code: {ret}")
                rclpy.shutdown()
                return

            sdk_py.setSDKLidarPowerOn(True)
            self.get_logger().info(f"Lidar ID: {sdk_py.getSDKLidarID()}")
            self.get_logger().info(f"Firmware: {sdk_py.getSDKFirmwareVersion()}")
            self.get_logger().info(f"Hardware: {sdk_py.getSDKHardwareVersion()}")
            self.get_logger().info(f"Model: {sdk_py.getSDKLidarModel()}")
            self.get_logger().info("Advanced mode initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Advanced init failed: {e}")
            rclpy.shutdown()

    def _init_polling_mode(self, port, baudrate):
        """使用 Dev 类初始化（轮询模式）"""
        self.driver = LidarDriver()
        if not self.driver.initialize(port, baudrate):
            self.get_logger().error("Device initialization failed, node will exit")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Polling mode initialized, port: {port}, baudrate: {baudrate}")

        self.timer = self.create_timer(1.0 / self.scan_freq, self._timer_callback)

        # 统计变量
        self.frame_count = 0
        self.point_count = 0
        self.last_stats_time = self.get_clock().now()

    def _error_callback(self, error_code):
        self.get_logger().warn(f"SDK error code: {error_code}")

    def _second_info_callback(self, stat_info):
        try:
            fps = stat_info.u64FPS
            valid = stat_info.iValid
            invalid = stat_info.iInvalid
            total = valid + invalid
            if total > 0:
                self.get_logger().info(f"FPS(sampling): {fps} pts/s, valid: {valid}, invalid: {invalid}, valid%: {100.0*valid/total:.1f}%")
            else:
                self.get_logger().info(f"FPS(sampling): {fps} pts/s, no points")
        except Exception as e:
            self.get_logger().error(f"Second info callback error: {e}")

    def _point_cloud_callback(self, point_list):
        """处理点云回调（高级模式）"""
        if not point_list:
            return

        points = []
        for p in point_list:
            if p.bValid:
                angle_rad = math.radians(p.dAngle)
                dist_m = p.u16Dist / 1000.0
                if self.range_min <= dist_m <= self.range_max:
                    points.append((angle_rad, dist_m))

        if not points:
            return

        points.sort(key=lambda x: x[0])
        angles = [pt[0] for pt in points]
        ranges = [pt[1] for pt in points]

        angle_min = angles[0]
        angle_max = angles[-1]
        angle_increment = (angle_max - angle_min) / (len(angles) - 1) if len(angles) > 1 else 0.0

        scan_msg = LaserScan()
        ts_ns = point_list[0].u64TimeStampNs if hasattr(point_list[0], 'u64TimeStampNs') else 0
        if ts_ns > 0:
            sec = ts_ns // 1_000_000_000
            nanosec = ts_ns % 1_000_000_000
            scan_msg.header.stamp = rclpy.time.Time(seconds=sec, nanoseconds=nanosec).to_msg()
        else:
            scan_msg.header.stamp = self.get_clock().now().to_msg()

        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_freq
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges

        self.publisher.publish(scan_msg)
        if self.debug_log:
            self.get_logger().debug(f"Published frame, points: {len(ranges)}")

    def _timer_callback(self):
        """轮询模式的定时回调"""
        data_list = self.driver.get_scan_data(self.reverse)
        if not data_list:
            if self.debug_log:
                self.get_logger().debug("No data received")
            return

        # 统计
        self.frame_count += 1
        self.point_count += len(data_list)
        now = self.get_clock().now()
        if (now - self.last_stats_time).nanoseconds > 1e9:
            self.get_logger().info(f"Publish freq: {self.frame_count} Hz, points/sec: {self.point_count}")
            self.frame_count = 0
            self.point_count = 0
            self.last_stats_time = now

        scan_msg = self._build_scan_msg(data_list)
        if scan_msg:
            self.publisher.publish(scan_msg)
            if self.debug_log:
                self.get_logger().debug(f"Published frame, points: {len(data_list)}")

    def _build_scan_msg(self, node_list):
        if not node_list:
            return None

        points = []
        for node in node_list:
            angle_deg = node.angle_q6_checkbit / 64.0
            angle_rad = math.radians(angle_deg)
            dist_mm = node.distance_q2 / 4.0
            dist_m = dist_mm / 1000.0
            if node.is_valid == 1 and self.range_min <= dist_m <= self.range_max:
                points.append((angle_rad, dist_m))

        if not points:
            return None

        points.sort(key=lambda x: x[0])
        angles = [pt[0] for pt in points]
        ranges = [pt[1] for pt in points]

        angle_min = angles[0]
        angle_max = angles[-1]
        angle_increment = (angle_max - angle_min) / (len(angles) - 1) if len(angles) > 1 else 0.0

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_freq
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges
        return scan_msg

    def destroy_node(self):
        if hasattr(self, 'driver'):
            self.driver.shutdown()
        if HAS_ADVANCED_API:
            try:
                sdk_py.hcSDKUnInit()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()