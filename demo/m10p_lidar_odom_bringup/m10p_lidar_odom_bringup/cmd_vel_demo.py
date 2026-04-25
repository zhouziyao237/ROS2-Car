#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelDemo(Node):
    """Repeatable low-speed motion for scan-matching verification.

    This is not obstacle avoidance. It only publishes a simple /cmd_vel test signal.
    """
    def __init__(self):
        super().__init__('cmd_vel_demo')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode', 'gentle_arc')
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('angular_speed', 0.10)
        self.declare_parameter('straight_time', 8.0)
        self.declare_parameter('turn_time', 1.5)
        self.declare_parameter('pause_time', 0.5)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.mode = self.get_parameter('mode').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.straight_time = float(self.get_parameter('straight_time').value)
        self.turn_time = float(self.get_parameter('turn_time').value)
        self.pause_time = float(self.get_parameter('pause_time').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Publishing demo motion to {self.cmd_vel_topic}; mode={self.mode}. This is not obstacle avoidance.')

    def sim_time(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def tick(self):
        t = self.sim_time()
        msg = Twist()

        if self.mode == 'forward':
            msg.linear.x = self.linear_speed
        elif self.mode == 'gentle_arc':
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
        elif self.mode == 'circle':
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed * 0.45
        else:
            cycle = self.straight_time + self.pause_time + self.turn_time + self.pause_time
            phase = t % cycle
            if phase < self.straight_time:
                msg.linear.x = self.linear_speed
            elif phase < self.straight_time + self.pause_time:
                pass
            elif phase < self.straight_time + self.pause_time + self.turn_time:
                msg.angular.z = self.angular_speed
            else:
                pass

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
