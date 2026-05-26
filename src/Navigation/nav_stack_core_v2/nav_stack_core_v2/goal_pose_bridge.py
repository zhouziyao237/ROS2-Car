#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.action_name = self.get_parameter('action_name').value
        self.client = ActionClient(self, NavigateToPose, self.action_name)
        self.sub = self.create_subscription(PoseStamped, self.goal_pose_topic, self.cb_goal, 10)
        self.get_logger().info(f'goal_pose_bridge: {self.goal_pose_topic} -> {self.action_name}')
    def cb_goal(self, msg):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.client.send_goal_async(goal)
        self.get_logger().info(f'Sent goal: frame={msg.header.frame_id}, x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

def main():
    rclpy.init()
    node = GoalPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
