#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from nav2_msgs.action import NavigateToPose


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/goal_markers')
        self.declare_parameter('action_name', '/navigate_to_pose')

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_cb,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.goal_count = 0

        self.get_logger().info(
            f'Goal bridge started: {self.goal_topic} -> {self.action_name}, '
            f'marker={self.marker_topic}'
        )

    def publish_marker(self, pose_msg: PoseStamped):
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = 'goal_pose_bridge'
        marker.id = self.goal_count
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose

        marker.scale.x = 0.45
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0

        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)

    def goal_cb(self, pose_msg: PoseStamped):
        self.goal_count += 1
        self.publish_marker(pose_msg)

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'NavigateToPose action server not available: {self.action_name}'
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        pass

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by navigate_to_pose.')
            return

        self.get_logger().info('Goal accepted by navigate_to_pose.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        self.get_logger().info(f'Goal finished with status={result.status}.')


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


if __name__ == '__main__':
    main()