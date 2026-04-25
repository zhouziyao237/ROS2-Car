#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def yaw_from_quat(q):
    # planar yaw from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomErrorMonitor(Node):
    """Compare /lidar/odom against Gazebo diff-drive odometry for quick verification."""
    def __init__(self):
        super().__init__('odom_error_monitor')
        self.declare_parameter('lidar_odom_topic', '/lidar/odom')
        self.declare_parameter('truth_odom_topic', '/odom_truth')
        self.declare_parameter('error_topic', '/lidar/odom_error')
        self.declare_parameter('log_period', 2.0)
        self.declare_parameter('align_initial_pose', True)

        self.lidar_topic = self.get_parameter('lidar_odom_topic').value
        self.truth_topic = self.get_parameter('truth_odom_topic').value
        self.error_topic = self.get_parameter('error_topic').value
        self.log_period = float(self.get_parameter('log_period').value)
        self.align_initial_pose = bool(self.get_parameter('align_initial_pose').value)

        self.lidar_msg = None
        self.truth_msg = None
        self.offset_ready = False
        self.dx0 = 0.0
        self.dy0 = 0.0
        self.dyaw0 = 0.0

        self.pub = self.create_publisher(Float64MultiArray, self.error_topic, 10)
        self.create_subscription(Odometry, self.lidar_topic, self.lidar_cb, 20)
        self.create_subscription(Odometry, self.truth_topic, self.truth_cb, 20)
        self.timer = self.create_timer(max(0.2, self.log_period), self.report)
        self.get_logger().info(f'Comparing {self.lidar_topic} against {self.truth_topic}')

    def lidar_cb(self, msg):
        self.lidar_msg = msg
        self.compute_and_publish()

    def truth_cb(self, msg):
        self.truth_msg = msg
        self.compute_and_publish()

    def get_pose2d(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        return float(p.x), float(p.y), yaw

    def compute_and_publish(self):
        if self.lidar_msg is None or self.truth_msg is None:
            return

        lx, ly, lyaw = self.get_pose2d(self.lidar_msg)
        tx, ty, tyaw = self.get_pose2d(self.truth_msg)

        if self.align_initial_pose and not self.offset_ready:
            self.dx0 = lx - tx
            self.dy0 = ly - ty
            self.dyaw0 = norm_angle(lyaw - tyaw)
            self.offset_ready = True

        tx_aligned = tx + self.dx0
        ty_aligned = ty + self.dy0
        tyaw_aligned = norm_angle(tyaw + self.dyaw0)

        ex = lx - tx_aligned
        ey = ly - ty_aligned
        eyaw = norm_angle(lyaw - tyaw_aligned)
        etrans = math.hypot(ex, ey)

        arr = Float64MultiArray()
        # [x_error, y_error, translation_error, yaw_error_rad]
        arr.data = [ex, ey, etrans, eyaw]
        self.pub.publish(arr)

        self.last_error = (ex, ey, etrans, eyaw)

    def report(self):
        err = getattr(self, 'last_error', None)
        if err is None:
            return
        ex, ey, et, eyaw = err
        self.get_logger().info(
            f'odom error: ex={ex:+.3f} m, ey={ey:+.3f} m, trans={et:.3f} m, yaw={math.degrees(eyaw):+.2f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
