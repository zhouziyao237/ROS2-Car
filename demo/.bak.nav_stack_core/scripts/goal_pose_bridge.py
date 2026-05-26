#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')

        self.goal_topic = self.declare_parameter('goal_topic', '/goal_pose').value
        self.action_name = self.declare_parameter('action_name', '/navigate_to_pose').value
        self.marker_topic = self.declare_parameter('marker_topic', '/goal_markers').value
        self.clear_topic = self.declare_parameter('clear_topic', '/clear_goal').value
        self.stop_topic = self.declare_parameter('stop_topic', '/cmd_vel_selected').value
        self.cancel_previous_goal = self.declare_parameter('cancel_previous_goal', True).value

        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._goal_seq = 0

        self._action_client = ActionClient(self, NavigateToPose, self.action_name)

        self._goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self._on_goal_pose,
            10
        )

        self._clear_sub = self.create_subscription(
            Empty,
            self.clear_topic,
            self._on_clear_goal,
            10
        )

        self._marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self._stop_pub = self.create_publisher(
            Twist,
            self.stop_topic,
            10
        )

        self.get_logger().info(
            f'Goal bridge started: {self.goal_topic} -> {self.action_name}, '
            f'marker={self.marker_topic}, clear={self.clear_topic}, stop={self.stop_topic}'
        )

    def _on_goal_pose(self, msg: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Action server not available: {self.action_name}')
            return

        if self.cancel_previous_goal:
            self._cancel_current_goal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self._goal_seq += 1
        self._publish_goal_marker(msg, self._goal_seq)

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

        self.get_logger().info(
            f'Sent goal: frame={msg.header.frame_id}, '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(f'Goal send failed: {ex}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by navigate_to_pose.')
            return

        with self._lock:
            self._current_goal_handle = goal_handle

        self.get_logger().info('Goal accepted by navigate_to_pose.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Goal result failed: {ex}')
            return

        with self._lock:
            self._current_goal_handle = None

        self.get_logger().info(f'Goal finished with status={result.status}')

    def _feedback_callback(self, feedback_msg):
        pass

    def _on_clear_goal(self, _msg: Empty):
        self.get_logger().warn('Clear goal requested.')

        self._cancel_current_goal()
        self._delete_all_markers()
        self._publish_stop()

    def _cancel_current_goal(self):
        with self._lock:
            goal_handle = self._current_goal_handle

        if goal_handle is None:
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            self.get_logger().warn('Cancel request sent to navigate_to_pose.')
        except Exception as ex:
            self.get_logger().warn(f'Cancel current goal failed: {ex}')

        with self._lock:
            self._current_goal_handle = None

    def _cancel_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().warn(
                f'Cancel response received, goals_canceling={len(response.goals_canceling)}'
            )
        except Exception as ex:
            self.get_logger().warn(f'Cancel response failed: {ex}')

    def _publish_stop(self):
        msg = Twist()
        self._stop_pub.publish(msg)
        self.get_logger().warn(f'Published zero Twist to {self.stop_topic}')

    def _delete_all_markers(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL

        arr = MarkerArray()
        arr.markers.append(marker)

        self._marker_pub.publish(arr)
        self.get_logger().warn(f'DELETEALL published to {self.marker_topic}')

    def _publish_goal_marker(self, pose_msg: PoseStamped, marker_id: int):
        self._delete_all_markers()

        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = 'goal_pose'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = pose_msg.pose

        marker.scale.x = 0.45
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        arr = MarkerArray()
        arr.markers.append(marker)
        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()