#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__("goal_pose_bridge")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("marker_topic", "/goal_markers")
        self.declare_parameter("cancel_previous_goal", True)

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.cancel_previous_goal = bool(self.get_parameter("cancel_previous_goal").value)

        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        self.current_goal_handle: Optional = None

        self.get_logger().info(
            f"Goal bridge started: {self.goal_topic} -> {self.action_name}, marker={self.marker_topic}"
        )

    def publish_goal_marker(self, pose: PoseStamped):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "goal_pose_bridge"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.45
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1

        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)

    def goal_cb(self, msg: PoseStamped):
        stamped_goal = PoseStamped()
        stamped_goal.header = msg.header
        stamped_goal.header.stamp = self.get_clock().now().to_msg()
        stamped_goal.pose = msg.pose

        self.publish_goal_marker(stamped_goal)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to_pose action server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = stamped_goal

        if self.cancel_previous_goal and self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by navigate_to_pose.")
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted by navigate_to_pose.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, _feedback):
        # Keep callback lightweight. Detailed feedback can still be viewed by ros2 action cli.
        return

    def result_cb(self, future):
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f"Goal finished with status={status}.")
        except Exception as exc:
            self.get_logger().error(f"Failed to get goal result: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
