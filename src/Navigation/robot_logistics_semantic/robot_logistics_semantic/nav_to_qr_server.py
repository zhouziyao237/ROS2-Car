#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from robot_logistics_interfaces.srv import NavToQR

class NavToQRServer(Node):
    def __init__(self):
        super().__init__('nav_to_qr_server')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('default_stop_distance', 1.2)
        self.map_frame = self.get_parameter('map_frame').value
        self.default_stop_distance = float(self.get_parameter('default_stop_distance').value)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.srv = self.create_service(NavToQR, '/nav/to_qr', self.nav_service_cb)
        self.get_logger().info('nav_to_qr_server ready')

    def nav_service_cb(self, request, response):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            response.nav_success = False; response.nav_msg = 'Nav2 action server not ready'; return response
        stop_dist = float(request.stop_distance) if float(request.stop_distance) > 0.01 else self.default_stop_distance
        tx = request.target_x - stop_dist * math.cos(request.target_yaw)
        ty = request.target_y - stop_dist * math.sin(request.target_yaw)
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(tx); goal.pose.pose.position.y = float(ty)
        goal.pose.pose.orientation.z = math.sin(float(request.target_yaw)/2.0)
        goal.pose.pose.orientation.w = math.cos(float(request.target_yaw)/2.0)
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_response_cb)
        response.nav_success = True
        response.nav_msg = f'nav goal sent to shelf approach: x={tx:.2f}, y={ty:.2f}, stop={stop_dist:.2f}'
        self.get_logger().info(response.nav_msg)
        return response

    def goal_response_cb(self, future):
        try: gh = future.result()
        except Exception as e:
            self.get_logger().error(f'NavToQR goal failed: {repr(e)}'); return
        if not gh.accepted:
            self.get_logger().error('NavToQR goal rejected'); return
        self.get_logger().info('NavToQR goal accepted')
        gh.get_result_async().add_done_callback(lambda f: self.get_logger().info(f'NavToQR result status={f.result().status}' if f.result() else 'NavToQR result none'))


def main(args=None):
    rclpy.init(args=args); node = NavToQRServer()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
