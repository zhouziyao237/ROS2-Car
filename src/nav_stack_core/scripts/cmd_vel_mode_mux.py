#!/usr/bin/env python3
import math

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelModeMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mode_mux')

        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('manual_topic_alt', '/red_standard_robot1/cmd_vel')
        self.declare_parameter('auto_topic', '/cmd_vel')
        self.declare_parameter('auto_topic_alt', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('chassis_mode_topic', '/chassis_control_mode')
        self.declare_parameter('chassis_mode_timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('default_mode', 'manual')
        self.declare_parameter('cancel_nav_on_manual', True)
        self.declare_parameter('navigate_to_pose_action_name', '/navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action_name', '/navigate_through_poses')

        self.manual_topic = self.get_parameter('manual_topic').value
        self.manual_topic_alt = self.get_parameter('manual_topic_alt').value
        self.auto_topic = self.get_parameter('auto_topic').value
        self.auto_topic_alt = self.get_parameter('auto_topic_alt').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.chassis_mode_topic = self.get_parameter('chassis_mode_topic').value
        self.chassis_mode_timeout = float(self.get_parameter('chassis_mode_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.requested_mode = str(self.get_parameter('default_mode').value).strip().lower()
        self.cancel_nav_on_manual = bool(self.get_parameter('cancel_nav_on_manual').value)
        self.navigate_to_pose_action_name = str(
            self.get_parameter('navigate_to_pose_action_name').value
        ).strip()
        self.navigate_through_poses_action_name = str(
            self.get_parameter('navigate_through_poses_action_name').value
        ).strip()
        self.mode = self.requested_mode
        self.chassis_mode = None
        self.last_chassis_mode_time = None

        self.manual_msg = Twist()
        self.auto_msg = Twist()
        self.auto_msg_alt = Twist()
        self.last_manual_time = None
        self.last_auto_time = None
        self.last_auto_alt_time = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.nav_cancel_clients = []
        self.register_nav_cancel_client(self.navigate_to_pose_action_name)
        self.register_nav_cancel_client(self.navigate_through_poses_action_name)

        self.create_subscription(Twist, self.manual_topic, self.manual_cb, 10)
        if isinstance(self.manual_topic_alt, str) and self.manual_topic_alt.strip():
            self.create_subscription(Twist, self.manual_topic_alt, self.manual_cb, 10)
        self.create_subscription(Twist, self.auto_topic, self.auto_cb, 10)
        if isinstance(self.auto_topic_alt, str) and self.auto_topic_alt.strip():
            self.create_subscription(Twist, self.auto_topic_alt, self.auto_alt_cb, 10)
        self.create_subscription(String, self.mode_topic, self.mode_cb, 10)
        if isinstance(self.chassis_mode_topic, str) and self.chassis_mode_topic.strip():
            self.create_subscription(String, self.chassis_mode_topic, self.chassis_mode_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info(
            f'CmdVel Mux started. mode={self.mode}, manual={self.manual_topic}, '
            f'manual_alt={self.manual_topic_alt}, auto={self.auto_topic}, '
            f'auto_alt={self.auto_topic_alt}, out={self.output_topic}, '
            f'chassis_mode_topic={self.chassis_mode_topic}, '
            f'cancel_nav_on_manual={self.cancel_nav_on_manual}'
        )

    @staticmethod
    def action_cancel_service_name(action_name: str) -> str:
        return f"{action_name.rstrip('/')}/_action/cancel_goal"

    def register_nav_cancel_client(self, action_name: str):
        if not action_name:
            return
        service_name = self.action_cancel_service_name(action_name)
        client = self.create_client(CancelGoal, service_name)
        self.nav_cancel_clients.append((action_name, client))

    def now_sec(self):
        t = self.get_clock().now().nanoseconds
        return t / 1e9

    def is_fresh(self, last_time):
        if last_time is None:
            return False
        return (self.now_sec() - last_time) <= self.cmd_timeout

    def is_chassis_mode_fresh(self):
        if self.last_chassis_mode_time is None:
            return False
        return (self.now_sec() - self.last_chassis_mode_time) <= self.chassis_mode_timeout

    @staticmethod
    def is_nonzero_twist(msg: Twist, eps: float = 1e-4) -> bool:
        return (
            abs(msg.linear.x) > eps or
            abs(msg.linear.y) > eps or
            abs(msg.linear.z) > eps or
            abs(msg.angular.x) > eps or
            abs(msg.angular.y) > eps or
            abs(msg.angular.z) > eps
        )

    def manual_cb(self, msg: Twist):
        self.manual_msg = msg
        self.last_manual_time = self.now_sec()

    def auto_cb(self, msg: Twist):
        self.auto_msg = msg
        self.last_auto_time = self.now_sec()

    def auto_alt_cb(self, msg: Twist):
        self.auto_msg_alt = msg
        self.last_auto_alt_time = self.now_sec()

    def mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in ['manual', 'auto']:
            self.get_logger().warn(f'Ignore invalid mode: {new_mode}')
            return

        if new_mode != self.requested_mode:
            self.requested_mode = new_mode
            self.refresh_effective_mode()

    def chassis_mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in ['manual', 'auto']:
            self.get_logger().warn(f'Ignore invalid chassis mode: {new_mode}')
            return

        self.chassis_mode = new_mode
        self.last_chassis_mode_time = self.now_sec()
        self.refresh_effective_mode()

    def get_effective_mode(self):
        chassis_fresh = self.is_chassis_mode_fresh() and self.chassis_mode is not None
        if chassis_fresh and self.chassis_mode == 'manual':
            return 'manual'
        return self.requested_mode

    def refresh_effective_mode(self):
        effective_mode = self.get_effective_mode()
        if effective_mode != self.mode:
            previous_mode = self.mode
            self.mode = effective_mode
            self.get_logger().info(
                f'Switched effective control mode to: {self.mode} '
                f'(requested={self.requested_mode}, chassis={self.chassis_mode})'
            )
            # 模式切换时先发一次零速度，避免残留命令
            self.pub.publish(Twist())
            if self.cancel_nav_on_manual and self.mode == 'manual' and previous_mode != 'manual':
                self.cancel_active_navigation_tasks()

    def cancel_active_navigation_tasks(self):
        cancel_sent = False
        for action_name, client in self.nav_cancel_clients:
            if not client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    f'Skip cancel for {action_name}: cancel service not available.'
                )
                continue

            # Empty CancelGoal request means "cancel all goals" on this action server.
            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(
                lambda future, action_name=action_name: self.cancel_done_cb(action_name, future)
            )
            cancel_sent = True

        if cancel_sent:
            self.get_logger().info(
                'Manual takeover detected. Requested cancellation of active navigation goals.'
            )

    def cancel_done_cb(self, action_name: str, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to cancel goals on {action_name}: {exc}')
            return

        self.get_logger().info(
            f'Cancel response from {action_name}: goals_canceling={len(response.goals_canceling)}.'
        )

    def timer_cb(self):
        self.refresh_effective_mode()
        out = Twist()

        if self.mode == 'manual':
            if self.is_fresh(self.last_manual_time):
                out = self.manual_msg
        else:  # auto
            auto_fresh = self.is_fresh(self.last_auto_time)
            auto_alt_fresh = self.is_fresh(self.last_auto_alt_time)

            # Prefer a non-zero command if available to avoid being masked by a zero stream.
            if auto_fresh and self.is_nonzero_twist(self.auto_msg):
                out = self.auto_msg
            elif auto_alt_fresh and self.is_nonzero_twist(self.auto_msg_alt):
                out = self.auto_msg_alt
            elif auto_fresh:
                out = self.auto_msg
            elif auto_alt_fresh:
                out = self.auto_msg_alt

        # 简单防 NaN
        for v in [
            out.linear.x, out.linear.y, out.linear.z,
            out.angular.x, out.angular.y, out.angular.z
        ]:
            if not math.isfinite(v):
                self.get_logger().warn('Non-finite cmd_vel detected, publish zero instead.')
                out = Twist()
                break

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
