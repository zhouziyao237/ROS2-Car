#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.twist = Twist()
        self.speed = 0.5
        self.turn = 1.0

    def get_key(self):
        """非阻塞获取单个键盘字符"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use W/A/S/D to move, X to stop, Q to quit")
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = 0.0
                elif key == 's':
                    self.twist.linear.x = -self.speed
                    self.twist.angular.z = 0.0
                elif key == 'a':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.turn
                elif key == 'd':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -self.turn
                elif key == 'x':
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                elif key == 'q':
                    break
                else:
                    continue
                self.publisher.publish(self.twist)
                self.get_logger().info(f"Publishing: linear.x={self.twist.linear.x:.2f}, angular.z={self.twist.angular.z:.2f}")
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            # 停止机器人
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            self.get_logger().info("Stopped")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()   # 假设你的节点类名为 KeyboardTeleop
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()