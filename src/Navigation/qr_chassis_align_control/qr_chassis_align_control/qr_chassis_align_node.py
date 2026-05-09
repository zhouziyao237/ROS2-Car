#!/usr/bin/env python3
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, String


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class QrChassisAlignNode(Node):
    def __init__(self):
        super().__init__('qr_chassis_align_node')

        self.declare_parameter('target_pose_topic', '/target_pose_base')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_manual')
        self.declare_parameter('enable_topic', '/qr_chassis_align/enable')
        self.declare_parameter('debug_topic', '/qr_chassis_align/debug')

        self.declare_parameter('enable_align_on_start', True)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('target_timeout_sec', 0.20)

        # 距离控制：默认用 point.x
        self.declare_parameter('distance_axis', 'x')
        self.declare_parameter('desired_distance_mm', 600.0)
        self.declare_parameter('deadband_distance_mm', 100.0)
        self.declare_parameter('kp_distance', 0.00018)
        self.declare_parameter('max_forward_speed', 0.06)
        self.declare_parameter('forward_direction', 1.0)

        # 是否启用左右修正；当前可以先 false
        self.declare_parameter('enable_side_control', False)
        self.declare_parameter('side_axis', 'y')
        self.declare_parameter('desired_side_mm', 0.0)
        self.declare_parameter('deadband_side_mm', 60.0)
        self.declare_parameter('kp_side', 0.00015)
        self.declare_parameter('max_side_speed', 0.05)
        self.declare_parameter('side_direction', -1.0)

        # 输出映射：通过 YAML 决定速度写入哪个字段
        # 可选：linear.x / linear.y / angular.z / none
        self.declare_parameter('forward_cmd_field', 'linear.x')
        self.declare_parameter('side_cmd_field', 'none')

        self.declare_parameter('filter_window', 3)
        self.declare_parameter('max_jump_distance_mm', 800.0)
        self.declare_parameter('max_jump_side_mm', 800.0)

        self.declare_parameter('verbose', True)

        self.enabled = bool(self.get_parameter('enable_align_on_start').value)

        win = max(1, int(self.get_parameter('filter_window').value))
        self.distance_buf = deque(maxlen=win)
        self.side_buf = deque(maxlen=win)

        self.last_distance_mm = None
        self.last_side_mm = None
        self.last_target_time = None

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('output_cmd_topic').value,
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            self.get_parameter('debug_topic').value,
            10
        )

        self.create_subscription(
            PointStamped,
            self.get_parameter('target_pose_topic').value,
            self.target_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('enable_topic').value,
            self.enable_cb,
            10
        )

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / max(1.0, rate), self.control_loop)

        self.log_debug(
            f'qr_chassis_align_node started enabled={self.enabled}, '
            f'output={self.get_parameter("output_cmd_topic").value}, '
            f'forward_cmd_field={self.get_parameter("forward_cmd_field").value}, '
            f'side_cmd_field={self.get_parameter("side_cmd_field").value}'
        )

    def log_debug(self, text):
        if not bool(self.get_parameter('verbose').value):
            return
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)
        self.get_logger().info(text)

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)
        self.clear_state()
        if not self.enabled:
            self.publish_zero('disabled')
        self.log_debug(f'enabled={self.enabled}')

    def clear_state(self):
        self.distance_buf.clear()
        self.side_buf.clear()
        self.last_distance_mm = None
        self.last_side_mm = None

    def get_axis_mm(self, msg, axis):
        axis = str(axis).lower()
        if axis == 'x':
            return float(msg.point.x) * 1000.0
        if axis == 'y':
            return float(msg.point.y) * 1000.0
        if axis == 'z':
            return float(msg.point.z) * 1000.0
        return float(msg.point.x) * 1000.0

    def target_cb(self, msg):
        distance_axis = self.get_parameter('distance_axis').value
        side_axis = self.get_parameter('side_axis').value

        distance_mm = self.get_axis_mm(msg, distance_axis)
        side_mm = self.get_axis_mm(msg, side_axis)

        if self.last_distance_mm is not None:
            max_jump = float(self.get_parameter('max_jump_distance_mm').value)
            if abs(distance_mm - self.last_distance_mm) > max_jump:
                self.log_debug(
                    f'HOLD distance jump {self.last_distance_mm:.1f}->{distance_mm:.1f}'
                )
                return

        if self.last_side_mm is not None:
            max_jump = float(self.get_parameter('max_jump_side_mm').value)
            if abs(side_mm - self.last_side_mm) > max_jump:
                self.log_debug(
                    f'HOLD side jump {self.last_side_mm:.1f}->{side_mm:.1f}'
                )
                return

        self.last_distance_mm = distance_mm
        self.last_side_mm = side_mm

        self.distance_buf.append(distance_mm)
        self.side_buf.append(side_mm)

        self.last_target_time = self.get_clock().now()

    def median(self, q):
        data = sorted(q)
        return data[len(data) // 2]

    def has_recent_target(self):
        if self.last_target_time is None:
            return False

        dt = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        return dt <= float(self.get_parameter('target_timeout_sec').value)

    def publish_zero(self, reason):
        self.cmd_pub.publish(Twist())
        self.log_debug(f'ZERO {reason}')

    def set_cmd_field(self, cmd, field, value):
        field = str(field).lower().strip()

        if field in ['', 'none', 'null', 'disable', 'disabled']:
            return

        if field == 'linear.x':
            cmd.linear.x = float(value)
            return

        if field == 'linear.y':
            cmd.linear.y = float(value)
            return

        if field == 'linear.z':
            cmd.linear.z = float(value)
            return

        if field == 'angular.z':
            cmd.angular.z = float(value)
            return

        self.get_logger().warn(f'Unknown cmd field: {field}')

    def control_loop(self):
        if not self.enabled:
            return

        if not self.has_recent_target():
            self.clear_state()
            self.publish_zero('target lost')
            return

        if len(self.distance_buf) < self.distance_buf.maxlen:
            self.publish_zero(
                f'waiting filter {len(self.distance_buf)}/{self.distance_buf.maxlen}'
            )
            return

        distance_mm = self.median(self.distance_buf)
        side_mm = self.median(self.side_buf) if len(self.side_buf) else 0.0

        desired_distance = float(self.get_parameter('desired_distance_mm').value)
        deadband_distance = float(self.get_parameter('deadband_distance_mm').value)
        kp_distance = float(self.get_parameter('kp_distance').value)
        max_forward = float(self.get_parameter('max_forward_speed').value)
        forward_direction = float(self.get_parameter('forward_direction').value)

        distance_err = distance_mm - desired_distance

        if abs(distance_err) <= deadband_distance:
            forward_cmd = 0.0
        else:
            forward_cmd = clamp(
                forward_direction * kp_distance * distance_err,
                -max_forward,
                max_forward
            )

        side_cmd = 0.0
        if bool(self.get_parameter('enable_side_control').value):
            desired_side = float(self.get_parameter('desired_side_mm').value)
            deadband_side = float(self.get_parameter('deadband_side_mm').value)
            kp_side = float(self.get_parameter('kp_side').value)
            max_side = float(self.get_parameter('max_side_speed').value)
            side_direction = float(self.get_parameter('side_direction').value)

            side_err = side_mm - desired_side

            if abs(side_err) <= deadband_side:
                side_cmd = 0.0
            else:
                side_cmd = clamp(
                    side_direction * kp_side * side_err,
                    -max_side,
                    max_side
                )
        else:
            side_err = 0.0

        cmd = Twist()

        self.set_cmd_field(
            cmd,
            self.get_parameter('forward_cmd_field').value,
            forward_cmd
        )

        self.set_cmd_field(
            cmd,
            self.get_parameter('side_cmd_field').value,
            side_cmd
        )

        self.cmd_pub.publish(cmd)

        self.log_debug(
            f'ALIGN distance={distance_mm:.1f} desired={desired_distance:.1f} '
            f'err={distance_err:.1f} forward_cmd={forward_cmd:.3f} '
            f'field={self.get_parameter("forward_cmd_field").value} '
            f'side={side_mm:.1f} side_cmd={side_cmd:.3f} '
            f'side_field={self.get_parameter("side_cmd_field").value}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = QrChassisAlignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()