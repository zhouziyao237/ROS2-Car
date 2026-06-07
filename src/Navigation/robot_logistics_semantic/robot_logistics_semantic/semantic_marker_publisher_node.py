#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish shelf markers from semantic_library.json."""

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray


class SemanticMarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('semantic_marker_publisher_node')
        self.declare_parameter('semantic_map_path', '/host-home/maps/semantic_library.json')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.semantic_map_path = Path(self.get_parameter('semantic_map_path').value)
        self.map_frame = self.get_parameter('map_frame').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.pub = self.create_publisher(MarkerArray, '/semantic/shelf_markers', 10)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_cb)
        self.get_logger().info('semantic_marker_publisher_node started')

    def timer_cb(self):
        data = self.load_library()
        arr = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        marker_id = 1
        for shelf_id, shelf in data.get('shelves', {}).items():
            pose = shelf.get('current_map_pose', {})
            if not pose.get('valid', False):
                continue
            x = float(pose.get('x', 0.0))
            y = float(pose.get('y', 0.0))
            yaw = float(pose.get('yaw', 0.0))
            geom = shelf.get('geometry', {})
            width = float(geom.get('width', 1.85))
            depth = float(geom.get('depth', 0.35))
            height = float(geom.get('height', 1.50))
            column_id = shelf.get('column_id', '')

            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = self.get_clock().now().to_msg()
            body.ns = 'semantic_shelves'
            body.id = marker_id; marker_id += 1
            body.type = Marker.CUBE
            body.action = Marker.ADD
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = height / 2.0
            body.pose.orientation = self.yaw_to_quat(yaw)
            # yaw = shelf front normal direction. x axis should be depth.
            # current_map_pose.yaw is shelf front outward normal.
            # Marker X axis = shelf depth direction.
            # Marker Y axis = shelf front long-face width direction.
            body.scale.x = depth
            body.scale.y = width
            body.scale.z = height
            body.color.r = 0.0
            body.color.g = 0.8
            body.color.b = 0.1
            body.color.a = 0.45
            arr.markers.append(body)

            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'semantic_shelf_front_direction'
            arrow.id = marker_id; marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = height + 0.15
            arrow.pose.orientation = self.yaw_to_quat(yaw)
            arrow.scale.x = 0.65
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            arrow.color.r = 1.0
            arrow.color.g = 0.1
            arrow.color.b = 0.1
            arrow.color.a = 0.95
            arr.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'semantic_shelf_labels'
            text.id = marker_id; marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = height + 0.45
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            layers = ','.join(sorted(shelf.get('layers', {}).keys()))
            source = pose.get('source', '')
            text.text = f'{shelf_id}  {column_id}\nlayers={layers}\n{source}'
            arr.markers.append(text)

            app_pose = shelf.get('approach_pose', {})
            if app_pose.get('valid', False):
                app = Marker()
                app.header.frame_id = self.map_frame
                app.header.stamp = self.get_clock().now().to_msg()
                app.ns = 'semantic_shelf_approach'
                app.id = marker_id; marker_id += 1
                app.type = Marker.ARROW
                app.action = Marker.ADD
                app.pose.position.x = float(app_pose.get('x', 0.0))
                app.pose.position.y = float(app_pose.get('y', 0.0))
                app.pose.position.z = 0.12
                app.pose.orientation = self.yaw_to_quat(float(app_pose.get('yaw', 0.0)))
                app.scale.x = 0.45
                app.scale.y = 0.05
                app.scale.z = 0.05
                app.color.r = 0.1
                app.color.g = 0.3
                app.color.b = 1.0
                app.color.a = 0.9
                arr.markers.append(app)

        self.pub.publish(arr)

    def load_library(self):
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

    def yaw_to_quat(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMarkerPublisherNode()
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
