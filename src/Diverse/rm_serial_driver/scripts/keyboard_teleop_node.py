#!/usr/bin/env python3
import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from auto_aim_interfaces.msg import PlatformTarget


HELP = """
================ Keyboard Teleop ================

Chassis:
  W/S : forward / backward
  A/D : left / right
  Q/E : rotate left / right
  O or SPACE : stop chassis

Platform:
  R/F : lift up / down
  T/G : slide forward / backward
  Y/H : screw extend / retract
  P   : reset platform target to zero

Other:
  I   : print current platform target
  ?   : print this help
  CTRL-C : quit

=================================================
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_selected')
        self.declare_parameter('platform_target_topic', '/platform/target')

        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)

        self.declare_parameter('lift_step_mm', 20.0)
        self.declare_parameter('slide_step_mm', 10.0)
        self.declare_parameter('screw_step_mm', 5.0)

        self.declare_parameter('help_print_interval_sec', 8.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.platform_target_topic = self.get_parameter('platform_target_topic').value

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)

        self.lift_step_mm = float(self.get_parameter('lift_step_mm').value)
        self.slide_step_mm = float(self.get_parameter('slide_step_mm').value)
        self.screw_step_mm = float(self.get_parameter('screw_step_mm').value)

        self.help_print_interval_sec = float(
            self.get_parameter('help_print_interval_sec').value
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.platform_pub = self.create_publisher(
            PlatformTarget,
            self.platform_target_topic,
            10
        )

        self.target_lift_z_mm = 0.0
        self.target_slide_x_mm = 0.0
        self.target_screw_x_mm = 0.0

        self.last_help_print_time = 0.0

        print(HELP)
        self.get_logger().info(f'cmd_vel_topic={self.cmd_vel_topic}')
        self.get_logger().info(f'platform_target_topic={self.platform_target_topic}')
        self.get_logger().info(
            f'steps: lift={self.lift_step_mm} mm, '
            f'slide={self.slide_step_mm} mm, '
            f'screw={self.screw_step_mm} mm'
        )

    def get_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.1)
        if dr:
            return sys.stdin.read(1)
        return ''

    def print_target(self):
        self.get_logger().info(
            f'current platform target: '
            f'lift={self.target_lift_z_mm:.1f} mm, '
            f'slide={self.target_slide_x_mm:.1f} mm, '
            f'screw={self.target_screw_x_mm:.1f} mm'
        )

    def publish_platform(self):
        msg = PlatformTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_lift_z_mm = float(self.target_lift_z_mm)
        msg.target_slide_x_mm = float(self.target_slide_x_mm)
        msg.target_screw_x_mm = float(self.target_screw_x_mm)

        self.platform_pub.publish(msg)
        self.print_target()

    def publish_twist(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}'
        )

    def maybe_print_help(self):
        now = time.time()
        if now - self.last_help_print_time >= self.help_print_interval_sec:
            print(HELP)
            self.last_help_print_time = now

    def run(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                self.maybe_print_help()

                key = self.get_key()
                if not key:
                    continue

                key = key.lower()

                if key == '\x03':
                    break

                # Chassis
                if key == 'w':
                    self.publish_twist(vx=self.speed)
                elif key == 's':
                    self.publish_twist(vx=-self.speed)
                elif key == 'a':
                    self.publish_twist(vy=self.speed)
                elif key == 'd':
                    self.publish_twist(vy=-self.speed)
                elif key == 'q':
                    self.publish_twist(wz=self.turn)
                elif key == 'e':
                    self.publish_twist(wz=-self.turn)
                elif key == 'o' or key == ' ':
                    self.publish_twist(0.0, 0.0, 0.0)

                # Platform incremental target control
                elif key == 'r':
                    self.target_lift_z_mm += self.lift_step_mm
                    self.publish_platform()
                elif key == 'f':
                    self.target_lift_z_mm -= self.lift_step_mm
                    self.publish_platform()

                elif key == 't':
                    self.target_slide_x_mm += self.slide_step_mm
                    self.publish_platform()
                elif key == 'g':
                    self.target_slide_x_mm -= self.slide_step_mm
                    self.publish_platform()

                elif key == 'y':
                    self.target_screw_x_mm += self.screw_step_mm
                    self.publish_platform()
                elif key == 'h':
                    self.target_screw_x_mm -= self.screw_step_mm
                    self.publish_platform()

                elif key == 'p':
                    self.target_lift_z_mm = 0.0
                    self.target_slide_x_mm = 0.0
                    self.target_screw_x_mm = 0.0
                    self.publish_platform()

                elif key == 'i':
                    self.print_target()

                elif key == '?':
                    print(HELP)
                    self.last_help_print_time = time.time()

                else:
                    print(HELP)
                    self.last_help_print_time = time.time()

        finally:
            self.publish_twist(0.0, 0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()