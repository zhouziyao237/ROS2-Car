#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool


class RvizRestrictRegionNode(Node):
    """
    RViz 探索限制区域节点。

    用法：
      1. RViz 选择 Publish Point
      2. 在 map 上依次点击多个点
      3. 节点发布：
           /explore/allowed_polygon
           /explore/allowed_polygon_marker
      4. frontier_explorer_node 只在 polygon 内选探索点

    显示：
      黄色点 = 已点击的角点
      蓝色线 = 限制区域边界
    """

    def __init__(self):
        super().__init__('rviz_restrict_region_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_points', 3)

        self.map_frame = self.get_parameter('map_frame').value
        self.min_points = int(self.get_parameter('min_points').value)

        self.points = []

        # transient_local：RViz 后添加 display 也能收到最近一次 marker
        self.marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_cb,
            10
        )

        self.clear_sub = self.create_subscription(
            Bool,
            '/explore/clear_allowed_polygon',
            self.clear_cb,
            10
        )

        self.poly_pub = self.create_publisher(
            PolygonStamped,
            '/explore/allowed_polygon',
            self.marker_qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/explore/allowed_polygon_marker',
            self.marker_qos
        )

        self.timer = self.create_timer(0.5, self.timer_cb)

        self.get_logger().info('rviz_restrict_region_node started')
        self.get_logger().info('Use RViz Publish Point to define allowed exploration polygon.')

    def clicked_point_cb(self, msg: PointStamped):
        x = float(msg.point.x)
        y = float(msg.point.y)

        self.points.append((x, y))

        self.get_logger().info(
            f'add allowed polygon point #{len(self.points)}: x={x:.2f}, y={y:.2f}'
        )

        self.publish_all()

    def clear_cb(self, msg: Bool):
        if msg.data:
            self.points = []
            self.get_logger().warn('allowed polygon cleared')
            self.publish_delete_all()

    def timer_cb(self):
        self.publish_all()

    def publish_all(self):
        if len(self.points) >= self.min_points:
            self.publish_polygon()
        self.publish_marker_array()

    def publish_polygon(self):
        poly = PolygonStamped()
        poly.header.frame_id = self.map_frame
        poly.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.points:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            poly.polygon.points.append(p)

        self.poly_pub.publish(poly)

    def publish_marker_array(self):
        arr = MarkerArray()

        # 1. 先删除旧 marker，避免残留
        delete = Marker()
        delete.header.frame_id = self.map_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = 'explore_allowed_polygon'
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        # 2. 显示点击点：黄色小球
        point_marker = Marker()
        point_marker.header.frame_id = self.map_frame
        point_marker.header.stamp = self.get_clock().now().to_msg()
        point_marker.ns = 'explore_allowed_polygon'
        point_marker.id = 1
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.scale.x = 0.16
        point_marker.scale.y = 0.16
        point_marker.scale.z = 0.16
        point_marker.color.r = 1.0
        point_marker.color.g = 0.8
        point_marker.color.b = 0.0
        point_marker.color.a = 1.0

        for x, y in self.points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.10
            point_marker.points.append(p)

        arr.markers.append(point_marker)

        # 3. 显示区域线框：蓝色线
        line_marker = Marker()
        line_marker.header.frame_id = self.map_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'explore_allowed_polygon'
        line_marker.id = 2
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.06
        line_marker.color.r = 0.0
        line_marker.color.g = 0.8
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0

        for x, y in self.points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.12
            line_marker.points.append(p)

        if len(self.points) >= 3:
            x0, y0 = self.points[0]
            p = Point()
            p.x = float(x0)
            p.y = float(y0)
            p.z = 0.12
            line_marker.points.append(p)

        arr.markers.append(line_marker)

        self.marker_pub.publish(arr)

    def publish_delete_all(self):
        arr = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'explore_allowed_polygon'
        marker.action = Marker.DELETEALL
        arr.markers.append(marker)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RvizRestrictRegionNode()

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
