#!/usr/bin/env python3
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from auto_aim_interfaces.msg import PlatformTarget

# Chassis motion mapping, normalized values.
KEY_ACTION = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
    'o': (0.0, 0.0, 0.0),
    ' ': (0.0, 0.0, 0.0),
}

SPEED_UP_KEY = '+'
SPEED_DOWN_KEY = '-'
TURN_UP_KEY = '\x1b[A'
TURN_DOWN_KEY = '\x1b[B'

LIFT_UP_KEY = 'r'
LIFT_DOWN_KEY = 'f'
SLIDE_OUT_KEY = 't'
SLIDE_IN_KEY = 'g'
SCREW_OUT_KEY = 'y'
SCREW_IN_KEY = 'h'
PLATFORM_RESET_KEY = 'p'

HELP_MSG = """
========================================
   Keyboard Teleop with Platform Control
========================================
Chassis Movement:
   W : forward      Q : rotate left
   S : backward     E : rotate right
   A : left move    O / SPACE : emergency stop
   D : right move

Chassis Speed:
   + / - : increase / decrease linear speed
   Up / Down Arrow : increase / decrease angular speed

Platform Position:
   R / F : lift up / down
   T / G : slide out / in
   Y / H : screw out / in
   P     : reset platform target to zero

Press CTRL-C to quit
========================================
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('cmd_vel_topic', '/red_standard_robot1/cmd_vel')
        self.declare_parameter('platform_target_topic', '/platform/target')
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('turn_step', 0.2)
        self.declare_parameter('lift_step_mm', 10.0)
        self.declare_parameter('slide_step_mm', 10.0)
        self.declare_parameter('screw_step_mm', 10.0)
        self.declare_parameter('lift_min_mm', -100000.0)
        self.declare_parameter('lift_max_mm', 100000.0)
        self.declare_parameter('slide_min_mm', -100000.0)
        self.declare_parameter('slide_max_mm', 100000.0)
        self.declare_parameter('screw_min_mm', -100000.0)
        self.declare_parameter('screw_max_mm', 100000.0)
        self.declare_parameter('platform_frame_id', 'base_link')
        self.declare_parameter('publish_period_sec', 0.1)

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.platform_target_topic = str(self.get_parameter('platform_target_topic').value)
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.turn_step = float(self.get_parameter('turn_step').value)
        self.lift_step_mm = float(self.get_parameter('lift_step_mm').value)
        self.slide_step_mm = float(self.get_parameter('slide_step_mm').value)
        self.screw_step_mm = float(self.get_parameter('screw_step_mm').value)
        self.lift_min_mm = float(self.get_parameter('lift_min_mm').value)
        self.lift_max_mm = float(self.get_parameter('lift_max_mm').value)
        self.slide_min_mm = float(self.get_parameter('slide_min_mm').value)
        self.slide_max_mm = float(self.get_parameter('slide_max_mm').value)
        self.screw_min_mm = float(self.get_parameter('screw_min_mm').value)
        self.screw_max_mm = float(self.get_parameter('screw_max_mm').value)
        self.platform_frame_id = str(self.get_parameter('platform_frame_id').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.platform_publisher = self.create_publisher(
            PlatformTarget, self.platform_target_topic, 10)

        self.key_file = None
        self.key_fd = None
        try:
            self.key_file = open('/dev/tty', 'rb', buffering=0)
            self.key_fd = self.key_file.fileno()
            self.settings = termios.tcgetattr(self.key_fd)
            self.get_logger().info("Keyboard input source: /dev/tty")
        except Exception:
            if sys.stdin.isatty():
                self.key_fd = sys.stdin.fileno()
                self.settings = termios.tcgetattr(self.key_fd)
                self.get_logger().info("Keyboard input source: stdin")
            else:
                raise RuntimeError(
                    "No interactive TTY found. Please run from an interactive terminal."
                )

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.target_lift_z_mm = 0.0
        self.target_slide_x_mm = 0.0
        self.target_screw_x_mm = 0.0

        self.get_logger().info(f"Publishing cmd_vel to '{self.cmd_vel_topic}'")
        self.get_logger().info(
            f"Publishing platform target to '{self.platform_target_topic}'")
        self.update_status_display()
        print(HELP_MSG)

        self.timer = self.create_timer(self.publish_period_sec, self.publish_outputs)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def update_status_display(self):
        self.get_logger().info(
            "Current speeds - linear: %.2f m/s, angular: %.2f rad/s | "
            "Platform target - lift: %.1f mm, slide: %.1f mm, screw: %.1f mm" % (
                self.speed, self.turn,
                self.target_lift_z_mm, self.target_slide_x_mm, self.target_screw_x_mm
            )
        )

    def get_key(self):
        tty.setraw(self.key_fd)
        rlist, _, _ = select.select([self.key_fd], [], [], 0.1)
        if rlist:
            key = os.read(self.key_fd, 1).decode(errors='ignore')
            if key == '\x1b':
                seq = key
                for _ in range(2):
                    rlist, _, _ = select.select([self.key_fd], [], [], 0.01)
                    if rlist:
                        seq += os.read(self.key_fd, 1).decode(errors='ignore')
                key = seq
        else:
            key = ''
        termios.tcsetattr(self.key_fd, termios.TCSADRAIN, self.settings)
        return key

    def publish_outputs(self):
        twist = Twist()
        twist.linear.x = self.linear_x * self.speed
        twist.linear.y = self.linear_y * self.speed
        twist.angular.z = self.angular_z * self.turn
        self.publisher.publish(twist)

        platform_msg = PlatformTarget()
        platform_msg.header.stamp = self.get_clock().now().to_msg()
        platform_msg.header.frame_id = self.platform_frame_id
        platform_msg.target_lift_z_mm = float(self.target_lift_z_mm)
        platform_msg.target_slide_x_mm = float(self.target_slide_x_mm)
        platform_msg.target_screw_x_mm = float(self.target_screw_x_mm)
        self.platform_publisher.publish(platform_msg)

    def handle_action_key(self, key):
        if key in KEY_ACTION:
            lx, ly, az = KEY_ACTION[key]
            self.linear_x = lx
            self.linear_y = ly
            self.angular_z = az

            if key in ('o', ' '):
                self.get_logger().warn("EMERGENCY STOP (O/SPACE pressed)")
            else:
                self.get_logger().info(
                    f"Move: linear.x={self.linear_x * self.speed:.2f}, "
                    f"linear.y={self.linear_y * self.speed:.2f}, "
                    f"angular.z={self.angular_z * self.turn:.2f}"
                )
            return True
        return False

    def handle_speed_key(self, key):
        updated = False
        if key == SPEED_UP_KEY:
            self.speed += self.speed_step
            updated = True
        elif key == SPEED_DOWN_KEY:
            self.speed = max(0.0, self.speed - self.speed_step)
            updated = True
        elif key == TURN_UP_KEY:
            self.turn += self.turn_step
            updated = True
        elif key == TURN_DOWN_KEY:
            self.turn = max(0.0, self.turn - self.turn_step)
            updated = True

        if updated:
            self.update_status_display()
        return updated

    def handle_platform_key(self, key):
        updated = False

        if key == LIFT_UP_KEY:
            self.target_lift_z_mm = self.clamp(
                self.target_lift_z_mm + self.lift_step_mm, self.lift_min_mm, self.lift_max_mm)
            updated = True
        elif key == LIFT_DOWN_KEY:
            self.target_lift_z_mm = self.clamp(
                self.target_lift_z_mm - self.lift_step_mm, self.lift_min_mm, self.lift_max_mm)
            updated = True
        elif key == SLIDE_OUT_KEY:
            self.target_slide_x_mm = self.clamp(
                self.target_slide_x_mm + self.slide_step_mm, self.slide_min_mm, self.slide_max_mm)
            updated = True
        elif key == SLIDE_IN_KEY:
            self.target_slide_x_mm = self.clamp(
                self.target_slide_x_mm - self.slide_step_mm, self.slide_min_mm, self.slide_max_mm)
            updated = True
        elif key == SCREW_OUT_KEY:
            self.target_screw_x_mm = self.clamp(
                self.target_screw_x_mm + self.screw_step_mm, self.screw_min_mm, self.screw_max_mm)
            updated = True
        elif key == SCREW_IN_KEY:
            self.target_screw_x_mm = self.clamp(
                self.target_screw_x_mm - self.screw_step_mm, self.screw_min_mm, self.screw_max_mm)
            updated = True
        elif key == PLATFORM_RESET_KEY:
            self.target_lift_z_mm = 0.0
            self.target_slide_x_mm = 0.0
            self.target_screw_x_mm = 0.0
            updated = True

        if updated:
            self.get_logger().info(
                "Platform target updated: lift=%.1f mm, slide=%.1f mm, screw=%.1f mm" % (
                    self.target_lift_z_mm,
                    self.target_slide_x_mm,
                    self.target_screw_x_mm,
                )
            )
            return True
        return False

    def spin(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if not key:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

                if self.handle_action_key(key):
                    pass
                elif self.handle_speed_key(key):
                    pass
                elif self.handle_platform_key(key):
                    pass
                elif key == '\x03':
                    break

                rclpy.spin_once(self, timeout_sec=0.01)

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        finally:
            twist = Twist()
            self.publisher.publish(twist)

            platform_msg = PlatformTarget()
            platform_msg.header.stamp = self.get_clock().now().to_msg()
            platform_msg.header.frame_id = self.platform_frame_id
            self.platform_publisher.publish(platform_msg)

            if self.key_fd is not None:
                termios.tcsetattr(self.key_fd, termios.TCSADRAIN, self.settings)
            if self.key_file is not None:
                self.key_file.close()
            self.get_logger().info("Keyboard Teleop Node stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
