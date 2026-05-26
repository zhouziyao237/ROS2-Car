#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task manager for final row-column parcel logic.

Task format:
  <column>-<layer>-<pickup_code>
  C3-L2-A1234 or 3-2-A1234

Execution:
  1. Parse target column, layer, pickup code.
  2. Find shelf in semantic_library.json where column_id == target column.
  3. Navigate to shelf.approach_pose.
  4. At shelf, confirm target layer QR and package QR in realtime.
  5. If target column shelf is not known, wait for exploration/registrar to discover it.
"""

import json
import math
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from std_msgs.msg import Bool
from robot_logistics_interfaces.srv import TaskAssign
from robot_logistics_interfaces.msg import SemanticObservation


class SemanticTaskManagerNode(Node):
    def __init__(self):
        super().__init__('semantic_task_manager_node')

        self.declare_parameter('semantic_map_path', '/host-home/maps/semantic_library.json')
        self.declare_parameter('task_service', '/task/assign')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('observation_topic', '/vision/semantic_observation')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('explore_enable_topic', '/explore/enable')
        self.declare_parameter('shelf_search_timeout_sec', 12.0)

        self.semantic_map_path = Path(self.get_parameter('semantic_map_path').value)
        self.task_service = self.get_parameter('task_service').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.observation_topic = self.get_parameter('observation_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.check_period_sec = float(self.get_parameter('check_period_sec').value)
        self.explore_enable_topic = self.get_parameter('explore_enable_topic').value
        self.shelf_search_timeout_sec = float(self.get_parameter('shelf_search_timeout_sec').value)

        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)
        self.create_service(TaskAssign, self.task_service, self.handle_task_assign)
        self.create_subscription(SemanticObservation, self.observation_topic, self.obs_cb, 30)
        self.status_pub = self.create_publisher(String, '/task/status', 10)
        self.explore_enable_pub = self.create_publisher(Bool, self.explore_enable_topic, 10)
        self.timer = self.create_timer(self.check_period_sec, self.timer_cb)

        self.state = 'IDLE'
        self.current_task = None
        self.current_shelf_id = ''
        self.arrived_search_start_time = None
        self.seen_layers = set()
        self.seen_packages = set()
        self.last_seen_shelf_id = ''

        self.get_logger().info('semantic_task_manager_node started')
        self.get_logger().info('task mode: target column -> shelf, then target layer + pickup QR')

    def handle_task_assign(self, request, response):
        task = self.parse_task_code(request.pick_code)
        if task is None:
            response.accept = False
            response.msg = 'invalid pick_code. Expected C3-L2-A1234 or 3-2-A1234'
            return response

        self.current_task = task
        self.reset_runtime_seen()
        self.state = 'FIND_TARGET_COLUMN'

        msg = (
            f'accepted task: column={task["column_id"]}, layer={task["layer_id"]}, '
            f'pickup_code={task["pickup_code"]}. Searching target column shelf.'
        )
        response.accept = True
        response.msg = msg
        self.get_logger().info(msg)
        self.publish_status(msg)
        self.try_start_navigation_to_target_column()
        return response

    def parse_task_code(self, raw):
        text = self.normalize_text(raw)
        m = re.fullmatch(r'(?:C)?(\d+)[-/:](?:L)?(\d+)[-/:]([A-Za-z0-9]+)', text, flags=re.IGNORECASE)
        if not m:
            return None
        col = int(m.group(1))
        row = int(m.group(2))
        return {
            'raw': raw,
            'col': col,
            'row': row,
            'column_id': f'C{col}',
            'layer_id': f'L{row}',
            'pickup_code': m.group(3).strip(),
        }

    def normalize_text(self, raw):
        text = str(raw).strip()
        m = re.fullmatch(r"b['\"](.+)['\"]", text)
        if m:
            text = m.group(1)
        text = text.replace(' ', '-')
        text = text.replace('_', '-')
        text = text.replace('，', '-')
        text = text.replace('－', '-')
        text = text.replace('\\', '-')
        return text

    def timer_cb(self):
        if self.current_task is None:
            return
        if self.state == 'WAITING_FOR_TARGET_COLUMN':
            self.try_start_navigation_to_target_column()
        elif self.state == 'SEARCHING_AT_SHELF':
            self.check_search_timeout()

    def try_start_navigation_to_target_column(self):
        if self.current_task is None:
            return False
        target_column = self.current_task['column_id']
        shelf_id, shelf = self.find_shelf_by_column(target_column)
        if shelf is None:
            self.state = 'WAITING_FOR_TARGET_COLUMN'
            self.set_explore_enabled(True)
            msg = (
                f'WAITING_FOR_TARGET_COLUMN: no shelf with column_id={target_column}. '
                'Keep frontier exploration running until that column QR is registered.'
            )
            self.get_logger().warn(msg)
            self.publish_status(msg)
            return False

        self.current_shelf_id = shelf_id
        return self.navigate_to_shelf(shelf_id, shelf)

    def obs_cb(self, obs: SemanticObservation):
        if self.current_task is None:
            return
        if self.state not in ['NAVIGATING_TO_SHELF', 'SEARCHING_AT_SHELF']:
            return

        # Only realtime confirmation here. Long-term registration is done by semantic_registrar_node.
        if obs.obs_type == 'layer_qr':
            if obs.layer_id:
                self.seen_layers.add(obs.layer_id)
            if obs.shelf_id:
                self.last_seen_shelf_id = obs.shelf_id
        elif obs.obs_type == 'package_qr':
            if obs.package_code:
                self.seen_packages.add(obs.package_code)
        elif obs.obs_type == 'column_qr':
            if obs.shelf_id:
                self.last_seen_shelf_id = obs.shelf_id

        self.check_target_found()

    def check_target_found(self):
        if self.current_task is None:
            return False
        target_layer = self.current_task['layer_id']
        target_pkg = self.current_task['pickup_code']

        has_layer = target_layer in self.seen_layers
        has_pkg = target_pkg in self.seen_packages

        if has_layer and has_pkg:
            msg = (
                f'FOUND_PACKAGE at shelf={self.current_shelf_id}. '
                f'target={self.current_task["column_id"]}-{target_layer}-{target_pkg}, '
                f'seen_layers={sorted(self.seen_layers)}, seen_packages={sorted(self.seen_packages)}'
            )
            self.state = 'FOUND_PACKAGE'
            self.get_logger().info(msg)
            self.publish_status(msg)
            return True
        return False

    def reset_runtime_seen(self):
        self.seen_layers = set()
        self.seen_packages = set()
        self.last_seen_shelf_id = ''
        self.arrived_search_start_time = None

    def find_shelf_by_column(self, column_id):
        data = self.load_semantic_library()
        shelves = data.get('shelves', {})
        for shelf_id, shelf in sorted(shelves.items()):
            if shelf.get('column_id', '') == column_id and self.shelf_has_valid_approach(shelf):
                return shelf_id, shelf
        return '', None

    def navigate_to_shelf(self, shelf_id, shelf):
        approach = shelf.get('approach_pose', {})
        if not approach.get('valid', False):
            msg = f'shelf {shelf_id} has no valid approach_pose'
            self.get_logger().warn(msg)
            self.publish_status(msg)
            self.state = 'WAITING_FOR_TARGET_COLUMN'
            self.set_explore_enabled(True)
            return False

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            msg = 'NavigateToPose action server not available'
            self.get_logger().error(msg)
            self.publish_status(msg)
            return False

        x = float(approach.get('x', 0.0))
        y = float(approach.get('y', 0.0))
        yaw = float(approach.get('yaw', 0.0))

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self.yaw_to_quat(yaw)

        self.reset_runtime_seen()
        self.state = 'NAVIGATING_TO_SHELF'
        self.set_explore_enabled(False)

        msg = (
            f'navigating to target column shelf={shelf_id}, column={self.current_task["column_id"]}, '
            f'approach_pose=({x:.2f},{y:.2f},{yaw:.2f}). '
            f'After arrival search layer={self.current_task["layer_id"]}, PKG:{self.current_task["pickup_code"]}'
        )
        self.get_logger().info(msg)
        self.publish_status(msg)

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback_cb)
        future.add_done_callback(lambda f: self.nav_goal_response_cb(f, shelf_id))
        return True

    def nav_goal_response_cb(self, future, shelf_id):
        try:
            goal_handle = future.result()
        except Exception as exc:
            msg = f'failed to send nav goal: {repr(exc)}'
            self.get_logger().error(msg)
            self.publish_status(msg)
            self.state = 'WAITING_FOR_TARGET_COLUMN'
            self.set_explore_enabled(True)
            return
        if not goal_handle.accepted:
            msg = f'Nav2 rejected goal for shelf={shelf_id}'
            self.get_logger().warn(msg)
            self.publish_status(msg)
            self.state = 'WAITING_FOR_TARGET_COLUMN'
            self.set_explore_enabled(True)
            return
        self.get_logger().info(f'Nav2 accepted goal for shelf={shelf_id}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.nav_result_cb(f, shelf_id))

    def nav_feedback_cb(self, feedback_msg):
        try:
            dist = feedback_msg.feedback.distance_remaining
            if dist > 0.05:
                self.get_logger().info(f'navigation feedback: distance_remaining={dist:.2f}')
        except Exception:
            pass

    def nav_result_cb(self, future, shelf_id):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            msg = f'nav result error: {repr(exc)}'
            self.get_logger().error(msg)
            self.publish_status(msg)
            self.state = 'WAITING_FOR_TARGET_COLUMN'
            self.set_explore_enabled(True)
            return

        if status == 4:
            self.state = 'SEARCHING_AT_SHELF'
            self.arrived_search_start_time = self.now_sec()
            msg = (
                f'arrived shelf={shelf_id}. Searching realtime QR: '
                f'{self.current_task["layer_id"]} and PKG:{self.current_task["pickup_code"]}'
            )
            self.get_logger().info(msg)
            self.publish_status(msg)
        else:
            msg = f'navigation to shelf={shelf_id} finished with status={status}'
            self.get_logger().warn(msg)
            self.publish_status(msg)
            self.state = 'WAITING_FOR_TARGET_COLUMN'
            self.set_explore_enabled(True)

    def check_search_timeout(self):
        if self.arrived_search_start_time is None:
            self.arrived_search_start_time = self.now_sec()
        elapsed = self.now_sec() - self.arrived_search_start_time
        status = (
            f'SEARCHING_AT_SHELF shelf={self.current_shelf_id}, elapsed={elapsed:.1f}s, '
            f'target={self.current_task["column_id"]}-{self.current_task["layer_id"]}-{self.current_task["pickup_code"]}, '
            f'seen_layers={sorted(self.seen_layers)}, seen_packages={sorted(self.seen_packages)}'
        )
        self.publish_status(status)
        if self.check_target_found():
            return
        if elapsed >= self.shelf_search_timeout_sec:
            msg = f'target not confirmed at shelf={self.current_shelf_id}; keep task active and continue searching.'
            self.get_logger().warn(msg)
            self.publish_status(msg)
            self.arrived_search_start_time = self.now_sec()

    def shelf_has_valid_approach(self, shelf):
        if shelf is None:
            return False
        return bool(shelf.get('approach_pose', {}).get('valid', False))

    def load_semantic_library(self):
        if not self.semantic_map_path.exists():
            return {'shelves': {}}
        try:
            text = self.semantic_map_path.read_text().strip()
            if not text:
                return {'shelves': {}}
            data = json.loads(text)
            if not isinstance(data, dict):
                return {'shelves': {}}
            data.setdefault('shelves', {})
            return data
        except Exception as exc:
            self.get_logger().warn(f'failed to load semantic library: {repr(exc)}')
            return {'shelves': {}}

    def set_explore_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.explore_enable_pub.publish(msg)
        self.get_logger().info(f'explore_enable={enabled}')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def now_sec(self):
        return float(self.get_clock().now().nanoseconds / 1e9)

    def yaw_to_quat(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SemanticTaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
