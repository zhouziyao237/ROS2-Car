#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import time

class MultiPointNavigator(Node):
    def __init__(self):
        super().__init__('jie_dian')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('等待导航动作服务器...')
        self._action_client.wait_for_server()
        self.get_logger().info('已连接到导航动作服务器')

        self.waypoints = [
            (1.0, 1.0, 0.0),
            (2.5, 3.0, 1.57),
            (4.0, 1.5, 3.14),
            (5.5, 4.0, -1.57),
        ]

        self.current_goal_handle = None
        self.nav_finished = False
        self.nav_result = GoalStatus.STATUS_UNKNOWN

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.get_logger().info(f'发送目标: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被服务器拒绝！')
            self.nav_finished = True
            self.nav_result = GoalStatus.STATUS_REJECTED
            return

        self.get_logger().info('目标已被接受，开始导航...')
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        status = future.result().status
        self.nav_finished = True
        self.nav_result = status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 成功到达目标点！')
        else:
            self.get_logger().error(f'❌ 导航失败，状态码: {status}')

    def wait_for_navigation(self):
        while rclpy.ok() and not self.nav_finished:
            rclpy.spin_once(self, timeout_sec=0.1)

    def simulate_pickup(self):
        self.get_logger().info('🔄 正在取物... (等待2秒)')
        time.sleep(2.0)
        self.get_logger().info('📦 物品已拾取！')

    def run_mission(self):
        self.get_logger().info('🚀 开始多点巡航任务')
        for idx, (x, y, yaw) in enumerate(self.waypoints, start=1):
            self.get_logger().info(f'📍 前往第 {idx} 个点位')
            self.nav_finished = False
            self.nav_result = GoalStatus.STATUS_UNKNOWN

            self.send_goal(x, y, yaw)
            self.wait_for_navigation()

            if self.nav_result == GoalStatus.STATUS_SUCCEEDED:
                self.simulate_pickup()
            else:
                self.get_logger().warn(f'跳过点位 {idx}，继续下一个任务')

            time.sleep(0.5)

        self.get_logger().info('🏁 所有巡航点位已完成，任务结束')

def main(args=None):
    rclpy.init(args=args)
    navigator = MultiPointNavigator()
    try:
        navigator.run_mission()
    except KeyboardInterrupt:
        navigator.get_logger().info('任务被用户中断')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()