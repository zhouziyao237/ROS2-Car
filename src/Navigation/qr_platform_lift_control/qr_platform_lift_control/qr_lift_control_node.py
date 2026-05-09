#!/usr/bin/env python3
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from rosidl_runtime_py.utilities import get_message


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class QrLiftControlNode(Node):
    def __init__(self):
        super().__init__('qr_lift_control_node')

        self.declare_parameter('target_pose_topic', '/target_pose_base')
        self.declare_parameter('platform_target_topic', '/platform/target')
        self.declare_parameter('enable_topic', '/qr_lift_control/enable')
        self.declare_parameter('debug_topic', '/qr_lift_control/debug')

        self.declare_parameter('platform_target_type', 'auto_aim_interfaces/msg/PlatformTarget')
        self.declare_parameter('lift_field', 'target_lift_z_mm')
        self.declare_parameter('slide_field', 'target_slide_x_mm')
        self.declare_parameter('screw_field', 'target_screw_x_mm')

        # 升降平台行程，不是离地高度
        self.declare_parameter('platform_min_height_mm', 0.0)
        self.declare_parameter('platform_max_height_mm', 280.0)
        self.declare_parameter('initial_lift_target_mm', 0.0)

        # 当前按你的实际效果：用 oak_camera_link 的 Z 控制升降
        self.declare_parameter('z_target_mm', 0.0)
        self.declare_parameter('kp_z', 0.20)
        self.declare_parameter('max_step_mm', 2.0)
        self.declare_parameter('deadband_mm', 8.0)
        self.declare_parameter('lift_direction', 1.0)

        # 数据过滤放宽：只防止特别离谱的数据
        self.declare_parameter('valid_abs_z_max_mm', 800.0)
        self.declare_parameter('valid_abs_y_max_mm', 800.0)

        self.declare_parameter('filter_window', 5)
        self.declare_parameter('stable_required', 2)
        self.declare_parameter('max_jump_z_mm', 800.0)

        # 丢目标自动回零
        self.declare_parameter('lost_timeout_sec', 0.5)
        self.declare_parameter('return_home_step_mm', 4.0)

        # 其他机构暂时保持
        self.declare_parameter('slide_hold_mm', 0.0)
        self.declare_parameter('screw_hold_mm', 0.0)

        self.enabled = True
        self.last_z = None
        self.stable_count = 0

        win = max(1, int(self.get_parameter('filter_window').value))
        self.x_buf = deque(maxlen=win)
        self.y_buf = deque(maxlen=win)
        self.z_buf = deque(maxlen=win)

        self.lift_cmd_mm = clamp(
            float(self.get_parameter('initial_lift_target_mm').value),
            float(self.get_parameter('platform_min_height_mm').value),
            float(self.get_parameter('platform_max_height_mm').value),
        )

        self.last_target_time = self.get_clock().now()

        msg_type = self.get_parameter('platform_target_type').value
        self.PlatformMsg = get_message(msg_type)

        self.create_subscription(
            PointStamped,
            self.get_parameter('target_pose_topic').value,
            self.pose_cb,
            10,
        )

        self.create_subscription(
            Bool,
            self.get_parameter('enable_topic').value,
            self.enable_cb,
            10,
        )

        self.platform_pub = self.create_publisher(
            self.PlatformMsg,
            self.get_parameter('platform_target_topic').value,
            10,
        )

        self.debug_pub = self.create_publisher(
            String,
            self.get_parameter('debug_topic').value,
            10,
        )

        self.create_timer(0.05, self.watchdog_cb)

        self.log_debug(f'qr_lift_control_node started, lift={self.lift_cmd_mm:.1f}')

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)
        self.log_debug(f'enabled={self.enabled}')

    def log_debug(self, text):
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)
        self.get_logger().info(text)

    def median(self, q):
        data = sorted(q)
        return data[len(data) // 2]

    def pose_cb(self, msg):
        if not self.enabled:
            return

        self.last_target_time = self.get_clock().now()

        x_mm = float(msg.point.x) * 1000.0
        y_mm = float(msg.point.y) * 1000.0
        z_mm = float(msg.point.z) * 1000.0

        z_max = float(self.get_parameter('valid_abs_z_max_mm').value)
        y_max = float(self.get_parameter('valid_abs_y_max_mm').value)

        if abs(z_mm) > z_max:
            self.stable_count = 0
            self.log_debug(
                f'HOLD z invalid z={z_mm:.1f}, x={x_mm:.1f}, y={y_mm:.1f}, lift={self.lift_cmd_mm:.1f}'
            )
            return

        if abs(y_mm) > y_max:
            self.stable_count = 0
            self.log_debug(
                f'HOLD y invalid y={y_mm:.1f}, x={x_mm:.1f}, z={z_mm:.1f}, lift={self.lift_cmd_mm:.1f}'
            )
            return

        if self.last_z is not None:
            max_jump_z = float(self.get_parameter('max_jump_z_mm').value)
            if abs(z_mm - self.last_z) > max_jump_z:
                self.stable_count = 0
                self.log_debug(
                    f'HOLD z jump {self.last_z:.1f}->{z_mm:.1f}, lift={self.lift_cmd_mm:.1f}'
                )
                return

        self.last_z = z_mm

        self.x_buf.append(x_mm)
        self.y_buf.append(y_mm)
        self.z_buf.append(z_mm)

        if len(self.z_buf) < self.z_buf.maxlen:
            self.log_debug(f'WAIT filter {len(self.z_buf)}/{self.z_buf.maxlen}')
            return

        self.stable_count += 1
        stable_required = int(self.get_parameter('stable_required').value)

        if self.stable_count < stable_required:
            self.log_debug(f'WAIT stable {self.stable_count}/{stable_required}')
            return

        x_f = self.median(self.x_buf)
        y_f = self.median(self.y_buf)
        z_f = self.median(self.z_buf)

        z_target = float(self.get_parameter('z_target_mm').value)
        err_z = z_f - z_target

        deadband = float(self.get_parameter('deadband_mm').value)
        kp = float(self.get_parameter('kp_z').value)
        max_step = float(self.get_parameter('max_step_mm').value)
        direction = float(self.get_parameter('lift_direction').value)

        if abs(err_z) < deadband:
            delta = 0.0
        else:
            delta = clamp(kp * err_z, -max_step, max_step)

        self.lift_cmd_mm = clamp(
            self.lift_cmd_mm + direction * delta,
            float(self.get_parameter('platform_min_height_mm').value),
            float(self.get_parameter('platform_max_height_mm').value),
        )

        self.publish_platform_target()

        self.log_debug(
            f'UPDATE x={x_f:.1f} y={y_f:.1f} z={z_f:.1f} '
            f'z_target={z_target:.1f} err_z={err_z:.1f} '
            f'delta={direction * delta:.1f} lift={self.lift_cmd_mm:.1f}'
        )

    def watchdog_cb(self):
        if not self.enabled:
            return

        now = self.get_clock().now()
        dt = (now - self.last_target_time).nanoseconds * 1e-9
        timeout = float(self.get_parameter('lost_timeout_sec').value)

        if dt < timeout:
            return

        if self.lift_cmd_mm <= 0.0:
            return

        step = float(self.get_parameter('return_home_step_mm').value)
        self.lift_cmd_mm = max(0.0, self.lift_cmd_mm - step)
        self.publish_platform_target()
        self.log_debug(f'LOST target, return home lift={self.lift_cmd_mm:.1f}')

    def publish_platform_target(self):
        msg = self.PlatformMsg()

        lift_field = self.get_parameter('lift_field').value
        slide_field = self.get_parameter('slide_field').value
        screw_field = self.get_parameter('screw_field').value

        if hasattr(msg, lift_field):
            setattr(msg, lift_field, float(self.lift_cmd_mm))
        else:
            self.get_logger().error(f'Platform msg has no lift field: {lift_field}')

        if hasattr(msg, slide_field):
            setattr(msg, slide_field, float(self.get_parameter('slide_hold_mm').value))

        if hasattr(msg, screw_field):
            setattr(msg, screw_field, float(self.get_parameter('screw_hold_mm').value))

        self.platform_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QrLiftControlNode()
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