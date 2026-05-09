#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, Float32, String


class MissionState(str, Enum):
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    TRACK = 'TRACK'
    READY_GRAB = 'READY_GRAB'
    PUSH = 'PUSH'
    DONE = 'DONE'
    LOST = 'LOST'


class AutoDecisionNode(Node):
    """Simple target-based automatic decision state machine.

    Input:
      /target_pose_base   geometry_msgs/PointStamped
        In current no-URDF mode frame_id is usually oak_camera_link:
          x forward, y left, z up, unit m.

    Outputs:
      /cmd_vel                  geometry_msgs/Twist
      /lift/target_delta_mm      std_msgs/Float32, relative lift correction
      /lift/target_height_mm     std_msgs/Float32, if /lift/current_height_mm exists
      /pusher/target_extend_mm   std_msgs/Float32
      /target_auto_decision/state std_msgs/String
      /target_auto_decision/ready_to_grab std_msgs/Bool
    """

    def __init__(self):
        super().__init__('auto_decision_node')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('target_pose_topic', '/target_pose_base')
        self.declare_parameter('state_topic', '/target_auto_decision/state')
        self.declare_parameter('ready_topic', '/target_auto_decision/ready_to_grab')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('lift_delta_topic', '/lift/target_delta_mm')
        self.declare_parameter('lift_target_topic', '/lift/target_height_mm')
        self.declare_parameter('lift_current_topic', '/lift/current_height_mm')
        self.declare_parameter('pusher_topic', '/pusher/target_extend_mm')
        self.declare_parameter('use_omni_lateral', False)
        self.declare_parameter('desired_x_mm', 350.0)
        self.declare_parameter('target_y_tolerance_mm', 25.0)
        self.declare_parameter('target_z_tolerance_mm', 35.0)
        self.declare_parameter('target_x_tolerance_mm', 40.0)
        self.declare_parameter('stable_required_count', 8)
        self.declare_parameter('target_timeout_sec', 0.60)
        self.declare_parameter('search_angular_z', 0.18)
        self.declare_parameter('max_linear_x', 0.10)
        self.declare_parameter('max_linear_y', 0.08)
        self.declare_parameter('max_angular_z', 0.35)
        self.declare_parameter('k_x', 0.00035)
        self.declare_parameter('k_y_linear', 0.00045)
        self.declare_parameter('k_y_yaw', 0.00120)
        self.declare_parameter('enable_lift_control', True)
        self.declare_parameter('lift_deadband_mm', 20.0)
        self.declare_parameter('lift_delta_limit_mm', 80.0)
        self.declare_parameter('pusher_extend_mm', 120.0)
        self.declare_parameter('push_hold_sec', 1.0)
        self.declare_parameter('stop_after_done', True)
        self.declare_parameter('debug_log', True)

        self.auto_enabled = bool(self.get_parameter('auto_start').value)
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.desired_x_mm = float(self.get_parameter('desired_x_mm').value)
        self.tol_x = float(self.get_parameter('target_x_tolerance_mm').value)
        self.tol_y = float(self.get_parameter('target_y_tolerance_mm').value)
        self.tol_z = float(self.get_parameter('target_z_tolerance_mm').value)
        self.stable_required_count = int(self.get_parameter('stable_required_count').value)

        self.use_omni_lateral = bool(self.get_parameter('use_omni_lateral').value)
        self.search_angular_z = float(self.get_parameter('search_angular_z').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.k_x = float(self.get_parameter('k_x').value)
        self.k_y_linear = float(self.get_parameter('k_y_linear').value)
        self.k_y_yaw = float(self.get_parameter('k_y_yaw').value)

        self.enable_lift_control = bool(self.get_parameter('enable_lift_control').value)
        self.lift_deadband_mm = float(self.get_parameter('lift_deadband_mm').value)
        self.lift_delta_limit_mm = float(self.get_parameter('lift_delta_limit_mm').value)
        self.pusher_extend_mm = float(self.get_parameter('pusher_extend_mm').value)
        self.push_hold_sec = float(self.get_parameter('push_hold_sec').value)
        self.stop_after_done = bool(self.get_parameter('stop_after_done').value)

        self.latest_target = None
        self.latest_target_stamp = None
        self.current_lift_mm = None
        self.stable_count = 0
        self.state = MissionState.SEARCH if self.auto_enabled else MissionState.IDLE
        self.push_start_time = None
        self.pusher_sent = False

        self.create_subscription(PointStamped, self.target_pose_topic, self.target_callback, 10)
        self.create_subscription(Bool, '/target_auto_decision/enable', self.enable_callback, 10)
        self.create_subscription(String, '/target_auto_decision/command', self.command_callback, 10)
        self.create_subscription(Float32, self.get_parameter('lift_current_topic').value, self.lift_current_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.lift_delta_pub = self.create_publisher(Float32, self.get_parameter('lift_delta_topic').value, 10)
        self.lift_target_pub = self.create_publisher(Float32, self.get_parameter('lift_target_topic').value, 10)
        self.pusher_pub = self.create_publisher(Float32, self.get_parameter('pusher_topic').value, 10)
        self.state_pub = self.create_publisher(String, self.get_parameter('state_topic').value, 10)
        self.ready_pub = self.create_publisher(Bool, self.get_parameter('ready_topic').value, 10)

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('auto_decision_node started')
        self.get_logger().info(f'target_pose_topic={self.target_pose_topic}')
        self.get_logger().info(f'initial_state={self.state.value}')

    def target_callback(self, msg: PointStamped):
        self.latest_target = msg
        self.latest_target_stamp = self.get_clock().now()

    def lift_current_callback(self, msg: Float32):
        self.current_lift_mm = float(msg.data)

    def enable_callback(self, msg: Bool):
        self.auto_enabled = bool(msg.data)
        if self.auto_enabled and self.state == MissionState.IDLE:
            self.set_state(MissionState.SEARCH)
        if not self.auto_enabled:
            self.set_state(MissionState.IDLE)
            self.stop_motion()

    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.auto_enabled = True
            self.reset_task()
            self.set_state(MissionState.SEARCH)
        elif cmd == 'stop':
            self.auto_enabled = False
            self.set_state(MissionState.IDLE)
            self.stop_motion()
        elif cmd == 'reset':
            self.reset_task()
            self.set_state(MissionState.SEARCH if self.auto_enabled else MissionState.IDLE)

    def reset_task(self):
        self.stable_count = 0
        self.push_start_time = None
        self.pusher_sent = False

    def update(self):
        self.publish_state()
        self.ready_pub.publish(Bool(data=self.state in [MissionState.READY_GRAB, MissionState.PUSH, MissionState.DONE]))

        if not self.auto_enabled or self.state == MissionState.IDLE:
            return

        has_target = self.has_fresh_target()
        if not has_target:
            self.stable_count = 0
            self.pusher_sent = False
            if self.state not in [MissionState.SEARCH, MissionState.DONE]:
                self.set_state(MissionState.LOST)
            if self.state in [MissionState.SEARCH, MissionState.LOST]:
                self.search_motion()
            return

        x_mm = self.latest_target.point.x * 1000.0
        y_mm = self.latest_target.point.y * 1000.0
        z_mm = self.latest_target.point.z * 1000.0

        if self.state in [MissionState.SEARCH, MissionState.LOST]:
            self.set_state(MissionState.TRACK)

        if self.state == MissionState.TRACK:
            self.track_target(x_mm, y_mm, z_mm)
            if self.is_aligned(x_mm, y_mm, z_mm):
                self.stable_count += 1
            else:
                self.stable_count = 0
            if self.stable_count >= self.stable_required_count:
                self.stop_motion()
                self.set_state(MissionState.READY_GRAB)

        elif self.state == MissionState.READY_GRAB:
            self.stop_motion()
            self.set_state(MissionState.PUSH)
            self.push_start_time = self.get_clock().now()

        elif self.state == MissionState.PUSH:
            self.stop_motion()
            if not self.pusher_sent:
                self.pusher_pub.publish(Float32(data=self.pusher_extend_mm))
                self.pusher_sent = True
                self.get_logger().info(f'pusher extend command: {self.pusher_extend_mm:.1f} mm')
            elapsed = (self.get_clock().now() - self.push_start_time).nanoseconds / 1e9
            if elapsed >= self.push_hold_sec:
                self.set_state(MissionState.DONE)

        elif self.state == MissionState.DONE:
            if self.stop_after_done:
                self.stop_motion()

    def has_fresh_target(self):
        if self.latest_target is None or self.latest_target_stamp is None:
            return False
        age = (self.get_clock().now() - self.latest_target_stamp).nanoseconds / 1e9
        return age <= self.target_timeout_sec

    def is_aligned(self, x_mm, y_mm, z_mm):
        return (
            abs(x_mm - self.desired_x_mm) <= self.tol_x and
            abs(y_mm) <= self.tol_y and
            abs(z_mm) <= self.tol_z
        )

    def track_target(self, x_mm, y_mm, z_mm):
        twist = Twist()

        # x_mm larger than desired means target is too far; move forward.
        err_x = x_mm - self.desired_x_mm
        if abs(err_x) > self.tol_x:
            twist.linear.x = self.clamp(self.k_x * err_x, -self.max_linear_x, self.max_linear_x)

        # y_mm > 0 means target is to the left of camera_link.
        # If chassis supports lateral movement, use linear.y.
        # Otherwise rotate toward the target.
        if abs(y_mm) > self.tol_y:
            if self.use_omni_lateral:
                twist.linear.y = self.clamp(self.k_y_linear * y_mm, -self.max_linear_y, self.max_linear_y)
            else:
                twist.angular.z = self.clamp(self.k_y_yaw * y_mm, -self.max_angular_z, self.max_angular_z)

        self.cmd_pub.publish(twist)
        self.publish_lift_command(z_mm)

        if self.debug_log:
            self.get_logger().info(
                f'TRACK x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm, '
                f'err_x={err_x:.1f}, cmd_vx={twist.linear.x:.3f}, cmd_wz={twist.angular.z:.3f}'
            )

    def publish_lift_command(self, z_mm):
        if not self.enable_lift_control:
            return
        if abs(z_mm) <= self.lift_deadband_mm:
            return
        delta = self.clamp(z_mm, -self.lift_delta_limit_mm, self.lift_delta_limit_mm)
        self.lift_delta_pub.publish(Float32(data=float(delta)))
        if self.current_lift_mm is not None:
            self.lift_target_pub.publish(Float32(data=float(self.current_lift_mm + delta)))

    def search_motion(self):
        twist = Twist()
        twist.angular.z = self.search_angular_z
        self.cmd_pub.publish(twist)

    def stop_motion(self):
        self.cmd_pub.publish(Twist())

    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'state: {self.state.value} -> {new_state.value}')
            self.state = new_state

    def publish_state(self):
        self.state_pub.publish(String(data=self.state.value))

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = AutoDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
