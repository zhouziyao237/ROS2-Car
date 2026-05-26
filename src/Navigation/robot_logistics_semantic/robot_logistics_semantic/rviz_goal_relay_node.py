#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class RvizGoalRelayNode(Node):
    def __init__(self):
        super().__init__('rviz_goal_relay_node')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, self.goal_pose_topic, self.goal_cb, 10)
        self.current_goal_handle = None
        self.get_logger().info(f'rviz_goal_relay_node started, subscribe {self.goal_pose_topic}')

    def goal_cb(self, msg):
        self.get_logger().info(f'new RViz goal: frame={msg.header.frame_id}, x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.cancel_current_goal()
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not ready')
            return
        goal = NavigateToPose.Goal(); goal.pose = msg
        if goal.pose.header.frame_id == '': goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_response_cb)
        self.get_logger().info('sent NavigateToPose goal')

    def cancel_current_goal(self):
        if self.current_goal_handle is None: return
        try:
            self.get_logger().info('cancel previous NavigateToPose goal')
            self.current_goal_handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn(f'cancel failed: {repr(e)}')
        self.current_goal_handle = None

    def goal_response_cb(self, future):
        try: gh = future.result()
        except Exception as e:
            self.get_logger().error(f'goal send failed: {repr(e)}'); return
        if not gh.accepted:
            self.get_logger().error('goal rejected by Nav2'); return
        self.current_goal_handle = gh
        self.get_logger().info('goal accepted by Nav2')
        gh.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'goal finished, status={result.status}')
        except Exception as e:
            self.get_logger().warn(f'goal result error: {repr(e)}')
        self.current_goal_handle = None

def main(args=None):
    rclpy.init(args=args); node = RvizGoalRelayNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
