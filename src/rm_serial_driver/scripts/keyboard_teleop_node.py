#!/usr/bin/env python3
import os
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 键盘动作映射（归一化值）
KEY_ACTION = {
    'w': (1.0, 0.0, 0.0),    # 前进
    's': (-1.0, 0.0, 0.0),   # 后退
    'a': (0.0, 1.0, 0.0),    # 左移
    'd': (0.0, -1.0, 0.0),   # 右移
    'q': (0.0, 0.0, 1.0),    # 左转
    'e': (0.0, 0.0, -1.0),   # 右转
    'o': (0.0, 0.0, 0.0),    # 急停 (O键)
    ' ': (0.0, 0.0, 0.0),    # 急停 (空格备用)
}

# 速度调节键映射
SPEED_UP_KEY = '+'      # 线速度增加
SPEED_DOWN_KEY = '-'    # 线速度减小
TURN_UP_KEY = '\x1b[A'  # 上箭头 (需要特殊处理)
TURN_DOWN_KEY = '\x1b[B' # 下箭头

HELP_MSG = """
========================================
   Keyboard Teleop with Speed Control
========================================
Movement:
   W : forward      Q : rotate left
   S : backward     E : rotate right
   A : left move    O / SPACE : emergency stop
   D : right move

Speed Control:
   + / - : increase / decrease linear speed
   ↑ / ↓ : increase / decrease angular speed

Press CTRL-C to quit
========================================
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # 声明参数
        self.declare_parameter('speed', 0.5)          # 初始线速度系数
        self.declare_parameter('turn', 1.0)           # 初始角速度系数
        self.declare_parameter('cmd_vel_topic', '/red_standard_robot1/cmd_vel')
        self.declare_parameter('speed_step', 0.1)     # 线速度调节步长
        self.declare_parameter('turn_step', 0.2)      # 角速度调节步长

        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.speed_step = self.get_parameter('speed_step').value
        self.turn_step = self.get_parameter('turn_step').value

        # 创建发布者
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 非阻塞键盘读取设置
        # ros2 launch 下 stdin 可能不是 TTY，这里优先使用 /dev/tty
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

        # 打印帮助信息
        self.get_logger().info(f"Publishing to '{self.cmd_vel_topic}'")
        self.update_status_display()
        print(HELP_MSG)

        # 定时器：每 0.1 秒发布一次当前速度（维持运动）
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # 当前归一化速度值 (-1.0 ~ 1.0)
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # 箭头键解析状态
        self.arrow_buffer = ''

    def update_status_display(self):
        """更新终端标题或打印状态（简单实现）"""
        self.get_logger().info(
            f"Current speeds - linear: {self.speed:.2f} m/s, angular: {self.turn:.2f} rad/s"
        )

    def get_key(self):
        """非阻塞读取键盘输入，支持多字节键（如箭头）"""
        tty.setraw(self.key_fd)
        rlist, _, _ = select.select([self.key_fd], [], [], 0.1)
        if rlist:
            key = os.read(self.key_fd, 1).decode(errors='ignore')
            # 处理转义序列（箭头键以 \x1b 开头）
            if key == '\x1b':
                # 尝试读取后续两个字符（超时很短）
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

    def publish_cmd(self):
        """根据当前速度值发布Twist消息"""
        twist = Twist()
        twist.linear.x = self.linear_x * self.speed
        twist.linear.y = self.linear_y * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_z * self.turn
        self.publisher.publish(twist)

    def handle_action_key(self, key):
        """处理运动/急停按键"""
        if key in KEY_ACTION:
            lx, ly, az = KEY_ACTION[key]
            self.linear_x = lx
            self.linear_y = ly
            self.angular_z = az

            if key == 'o' or key == ' ':
                self.get_logger().warn("EMERGENCY STOP (O/SPACE pressed)")
            else:
                self.get_logger().info(
                    f"Move: linear.x={self.linear_x*self.speed:.2f}, "
                    f"linear.y={self.linear_y*self.speed:.2f}, "
                    f"angular.z={self.angular_z*self.turn:.2f}"
                )
            return True
        return False

    def handle_speed_key(self, key):
        """处理速度调节按键"""
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

    def spin(self):
        """主循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if not key:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

                # 处理动作按键
                if self.handle_action_key(key):
                    pass
                # 处理速度调节
                elif self.handle_speed_key(key):
                    pass
                # 退出
                elif key == '\x03':  # Ctrl+C
                    break

                rclpy.spin_once(self, timeout_sec=0.01)

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        finally:
            # 停止机器人
            twist = Twist()
            self.publisher.publish(twist)
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
