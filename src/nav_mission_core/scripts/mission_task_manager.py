#!/usr/bin/env python3

from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from std_msgs.msg import Empty, String, Int32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


class MissionState(Enum):
    IDLE = 0
    NAV_TO_ANCHOR = 1
    SEARCHING = 2
    RETURN_TO_ANCHOR = 3
    FINISHED = 4


class MissionTaskManager(Node):
    def __init__(self):
        super().__init__('mission_task_manager')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_mode_topic', '/control_mode')
        self.declare_parameter('auto_mode_value', 'auto')
        self.declare_parameter('search_mode_value', 'search')
        self.declare_parameter('navigate_action_name', '/navigate_to_pose')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.control_mode_topic = self.get_parameter('control_mode_topic').value
        self.auto_mode_value = self.get_parameter('auto_mode_value').value
        self.search_mode_value = self.get_parameter('search_mode_value').value
        self.navigate_action_name = self.get_parameter('navigate_action_name').value

        self.state = MissionState.IDLE
        self.current_index = 0
        self.zones = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action_name)

        self.mode_pub = self.create_publisher(String, self.control_mode_topic, 10)
        self.start_search_pub = self.create_publisher(Empty, '/start_shelf_search', 10)
        self.current_zone_pub = self.create_publisher(Int32, '/current_zone_id', 10)
        self.current_anchor_pub = self.create_publisher(PoseStamped, '/current_anchor_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        self.create_subscription(PoseStamped, '/waypoint_pose', self.waypoint_callback, 10)
        self.create_subscription(Int32, '/tag_zone_id', self.tag_zone_callback, 10)
        self.create_subscription(Empty, '/start_mission', self.start_mission_callback, 10)
        self.create_subscription(Empty, '/clear_mission', self.clear_mission_callback, 10)
        self.create_subscription(Empty, '/search_done', self.search_done_callback, 10)
        self.create_subscription(Empty, '/skip_current_zone', self.skip_current_zone_callback, 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('mission_task_manager started.')
        self.get_logger().info('RViz2 SetGoal topic should be /waypoint_pose.')
        self.get_logger().info('Start mission: ros2 topic pub /start_mission std_msgs/msg/Empty "{}" -1')

    def timer_callback(self):
        self.publish_markers()
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def set_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Control mode set to: {mode}')

    def make_pose_from_robot_current_tf(self) -> PoseStamped:
        trans = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time()
        )

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = trans.transform.rotation
        return pose

    def waypoint_callback(self, msg: PoseStamped):
        if self.state not in [MissionState.IDLE, MissionState.FINISHED]:
            self.get_logger().warn('Mission is running. Ignore new RViz waypoint.')
            return

        if self.state == MissionState.FINISHED:
            self.state = MissionState.IDLE

        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        zone_id = len(self.zones) + 1
        self.zones.append({
            'zone_id': zone_id,
            'anchor_pose': msg,
            'source': 'rviz'
        })

        self.get_logger().info(
            f'Add RViz zone {zone_id}: '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

    def tag_zone_callback(self, msg: Int32):
        if self.state not in [MissionState.IDLE, MissionState.FINISHED]:
            self.get_logger().warn('Mission is running. Ignore new tag zone.')
            return

        if self.state == MissionState.FINISHED:
            self.state = MissionState.IDLE

        try:
            anchor_pose = self.make_pose_from_robot_current_tf()
        except Exception as e:
            self.get_logger().error(f'Failed to register tag zone by TF: {e}')
            return

        zone_id = int(msg.data)

        for zone in self.zones:
            if zone['zone_id'] == zone_id:
                zone['anchor_pose'] = anchor_pose
                zone['source'] = 'tag'
                self.get_logger().info(f'Update tag zone {zone_id} anchor pose.')
                return

        self.zones.append({
            'zone_id': zone_id,
            'anchor_pose': anchor_pose,
            'source': 'tag'
        })

        self.get_logger().info(
            f'Register tag zone {zone_id}: '
            f'x={anchor_pose.pose.position.x:.2f}, '
            f'y={anchor_pose.pose.position.y:.2f}'
        )

    def start_mission_callback(self, msg: Empty):
        if self.state not in [MissionState.IDLE, MissionState.FINISHED]:
            self.get_logger().warn(f'Mission already running. State: {self.state.name}')
            return

        if len(self.zones) == 0:
            self.get_logger().warn('No zone available. Add /waypoint_pose or /tag_zone_id first.')
            return

        self.zones.sort(key=lambda z: z['zone_id'])
        self.current_index = 0
        self.set_mode(self.auto_mode_value)
        self.navigate_to_current_anchor()

    def clear_mission_callback(self, msg: Empty):
        if self.state not in [MissionState.IDLE, MissionState.FINISHED]:
            self.get_logger().warn('Cannot clear mission while running.')
            return

        self.zones.clear()
        self.current_index = 0
        self.state = MissionState.IDLE
        self.publish_delete_all_markers()
        self.get_logger().info('Mission zones cleared.')

    def skip_current_zone_callback(self, msg: Empty):
        if self.state == MissionState.IDLE:
            self.get_logger().warn('No active zone to skip.')
            return

        self.get_logger().warn('Skip current zone requested.')
        self.current_index += 1

        if self.current_index >= len(self.zones):
            self.finish_mission()
        else:
            self.set_mode(self.auto_mode_value)
            self.navigate_to_current_anchor()

    def current_zone(self):
        if self.current_index < 0 or self.current_index >= len(self.zones):
            return None
        return self.zones[self.current_index]

    def publish_current_zone_info(self):
        zone = self.current_zone()
        if zone is None:
            return

        id_msg = Int32()
        id_msg.data = int(zone['zone_id'])
        self.current_zone_pub.publish(id_msg)

        anchor = zone['anchor_pose']
        anchor.header.stamp = self.get_clock().now().to_msg()
        self.current_anchor_pub.publish(anchor)

    def navigate_to_current_anchor(self):
        zone = self.current_zone()
        if zone is None:
            self.finish_mission()
            return

        self.state = MissionState.NAV_TO_ANCHOR
        self.publish_current_zone_info()

        goal_pose = zone['anchor_pose']
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            f'Navigate to zone {zone["zone_id"]}: '
            f'x={goal_pose.pose.position.x:.2f}, '
            f'y={goal_pose.pose.position.y:.2f}'
        )

        self.nav_client.wait_for_server()

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def return_to_current_anchor(self):
        zone = self.current_zone()
        if zone is None:
            self.finish_mission()
            return

        self.state = MissionState.RETURN_TO_ANCHOR
        self.set_mode(self.auto_mode_value)
        self.publish_current_zone_info()

        goal_pose = zone['anchor_pose']
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            f'Return to zone {zone["zone_id"]}: '
            f'x={goal_pose.pose.position.x:.2f}, '
            f'y={goal_pose.pose.position.y:.2f}'
        )

        self.nav_client.wait_for_server()

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected.')
            self.state = MissionState.IDLE
            return

        self.get_logger().info('Nav2 goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m',
            throttle_duration_sec=2.0
        )

    def nav_result_callback(self, future):
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Navigation failed. Status: {status}')
            self.state = MissionState.IDLE
            return

        if self.state == MissionState.NAV_TO_ANCHOR:
            self.get_logger().info('Arrived zone anchor. Start shelf search.')
            self.start_search()
            return

        if self.state == MissionState.RETURN_TO_ANCHOR:
            zone = self.current_zone()
            if zone is not None:
                self.get_logger().info(f'Returned to zone {zone["zone_id"]}.')

            self.current_index += 1

            if self.current_index >= len(self.zones):
                self.finish_mission()
            else:
                self.navigate_to_current_anchor()

    def start_search(self):
        self.state = MissionState.SEARCHING
        self.set_mode(self.search_mode_value)
        self.start_search_pub.publish(Empty())

        zone = self.current_zone()
        if zone is not None:
            self.get_logger().info(f'Search started at zone {zone["zone_id"]}.')

    def search_done_callback(self, msg: Empty):
        if self.state != MissionState.SEARCHING:
            self.get_logger().warn(f'Ignore /search_done. Current state: {self.state.name}')
            return

        self.get_logger().info('Search done. Returning to anchor.')
        self.return_to_current_anchor()

    def finish_mission(self):
        self.state = MissionState.FINISHED
        self.set_mode(self.auto_mode_value)
        self.get_logger().info('Mission finished. All zones processed.')

    def publish_delete_all_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def make_text_marker(self, marker_id, text, pose, z_offset):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mission_text'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z += z_offset
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, zone in enumerate(self.zones):
            pose = zone['anchor_pose']

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'mission_zones'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.pose.position.z = 0.08
            sphere.scale.x = 0.28
            sphere.scale.y = 0.28
            sphere.scale.z = 0.28

            if i == self.current_index and self.state not in [MissionState.IDLE, MissionState.FINISHED]:
                sphere.color.r = 1.0
                sphere.color.g = 0.2
                sphere.color.b = 0.1
                sphere.color.a = 1.0
            else:
                sphere.color.r = 0.1
                sphere.color.g = 0.8
                sphere.color.b = 1.0
                sphere.color.a = 0.9

            marker_array.markers.append(sphere)

            text = self.make_text_marker(
                1000 + i,
                f'Zone {zone["zone_id"]}\n{zone["source"]}',
                pose,
                0.45
            )
            marker_array.markers.append(text)

        if self.current_zone() is not None and self.state not in [MissionState.IDLE, MissionState.FINISHED]:
            pose = self.current_zone()['anchor_pose']

            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'mission_current'
            arrow.id = 5000
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose.pose
            arrow.pose.position.z = 0.15
            arrow.scale.x = 0.55
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.r = 1.0
            arrow.color.g = 0.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            state_text = self.make_text_marker(
                6000,
                f'STATE: {self.state.name}',
                pose,
                0.85
            )
            marker_array.markers.append(state_text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionTaskManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
