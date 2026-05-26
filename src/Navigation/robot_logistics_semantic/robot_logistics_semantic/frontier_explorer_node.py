#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import tf2_ros


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('enable_topic', '/explore/enable')
        self.declare_parameter('status_topic', '/explore/status')
        self.declare_parameter('allowed_polygon_topic', '/explore/allowed_polygon')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('goal_period_sec', 3.0)
        self.declare_parameter('goal_timeout_sec', 35.0)
        self.declare_parameter('min_frontier_cluster_size', 3)
        self.declare_parameter('min_frontier_distance', 0.45)
        self.declare_parameter('max_frontier_distance', 8.0)
        self.declare_parameter('frontier_blacklist_radius', 0.50)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('free_threshold', 10)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('info_gain_weight', 0.04)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('prefer_forward_weight', 0.35)
        self.declare_parameter('goal_safety_radius_m', 0.25)

        self.map_topic = self.get_parameter('map_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.allowed_polygon_topic = self.get_parameter('allowed_polygon_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.goal_period_sec = float(self.get_parameter('goal_period_sec').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.min_cluster = int(self.get_parameter('min_frontier_cluster_size').value)
        self.min_frontier_distance = float(self.get_parameter('min_frontier_distance').value)
        self.max_frontier_distance = float(self.get_parameter('max_frontier_distance').value)
        self.blacklist_radius = float(self.get_parameter('frontier_blacklist_radius').value)
        self.unknown_value = int(self.get_parameter('unknown_value').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.info_gain_weight = float(self.get_parameter('info_gain_weight').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)
        self.prefer_forward_weight = float(self.get_parameter('prefer_forward_weight').value)
        self.goal_safety_radius_m = float(self.get_parameter('goal_safety_radius_m').value)

        self.enabled = False
        self.map_msg = None
        self.allowed_polygon = []
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = 0.0
        self.last_goal_time = 0.0
        self.current_goal_xy = None
        self.blacklist = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 10)
        self.create_subscription(Bool, self.enable_topic, self.enable_cb, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.get_logger().info('frontier_explorer_node started')

    def publish_status(self, text):
        m = String(); m.data = text; self.status_pub.publish(m)

    def map_cb(self, msg):
        self.map_msg = msg

    def allowed_polygon_cb(self, msg):
        self.allowed_polygon = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        self.get_logger().info(f'allowed polygon updated: {len(self.allowed_polygon)} points')

    def enable_cb(self, msg):
        new_state = bool(msg.data)
        if new_state == self.enabled:
            return
        self.enabled = new_state
        if self.enabled:
            self.get_logger().info('frontier exploration enabled')
            self.publish_status('enabled')
            self.last_goal_time = 0.0
        else:
            self.get_logger().info('frontier exploration disabled')
            self.publish_status('disabled')
            self.cancel_goal()

    def timer_cb(self):
        if not self.enabled:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Nav2 navigate_to_pose action server not ready')
            self.publish_status('waiting_nav2')
            return
        if self.map_msg is None:
            self.get_logger().warn('waiting for /map')
            self.publish_status('waiting_map')
            return
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            self.publish_status('waiting_tf')
            return
        now = time.time()
        if self.goal_active:
            if now - self.goal_sent_time > self.goal_timeout_sec:
                self.get_logger().warn('frontier goal timeout, cancel and blacklist')
                if self.current_goal_xy is not None:
                    self.blacklist.append(self.current_goal_xy)
                self.cancel_goal()
            return
        if now - self.last_goal_time < self.goal_period_sec:
            return
        goal = self.choose_frontier_goal(robot_pose)
        if goal is None:
            self.get_logger().warn('no valid frontier found')
            self.publish_status('no_frontier')
            return
        self.send_nav_goal(goal, robot_pose)

    def lookup_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.3))
            q = tf.transform.rotation
            yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
            return tf.transform.translation.x, tf.transform.translation.y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {repr(e)}')
            return None

    def choose_frontier_goal(self, robot_pose):
        rx, ry, ryaw = robot_pose
        msg = self.map_msg
        width, height = msg.info.width, msg.info.height
        data = list(msg.data)
        if width < 10 or height < 10:
            return None
        frontier_cells = self.find_frontier_cells(data, width, height)
        clusters = self.cluster_frontiers(frontier_cells)
        candidates = []
        for cluster in clusters:
            if len(cluster) < self.min_cluster:
                continue
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            wx, wy = self.grid_to_world(cx, cy, msg)
            dist = math.hypot(wx - rx, wy - ry)
            if dist < self.min_frontier_distance or dist > self.max_frontier_distance:
                continue
            if self.is_blacklisted(wx, wy) or not self.is_goal_cell_safe(wx, wy):
                continue
            heading_error = abs(self.normalize_angle(math.atan2(wy - ry, wx - rx) - ryaw))
            score = self.distance_weight * dist - self.info_gain_weight * len(cluster) + self.prefer_forward_weight * heading_error
            candidates.append((score, wx, wy, len(cluster), dist))
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        score, wx, wy, size, dist = candidates[0]
        self.get_logger().info(f'chosen frontier: x={wx:.2f}, y={wy:.2f}, cluster={size}, dist={dist:.2f}, score={score:.2f}')
        self.publish_status('goal_selected')
        return wx, wy

    def find_frontier_cells(self, data, width, height):
        frontiers = set()
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.is_free(data, x, y, width) and self.has_unknown_neighbor(data, x, y, width, height):
                    frontiers.add((x, y))
        return frontiers

    def cluster_frontiers(self, frontier_cells):
        visited = set(); clusters = []
        for cell in frontier_cells:
            if cell in visited:
                continue
            q = deque([cell]); visited.add(cell); cluster = []
            while q:
                cx, cy = q.popleft(); cluster.append((cx, cy))
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                    nc = (cx+dx, cy+dy)
                    if nc in frontier_cells and nc not in visited:
                        visited.add(nc); q.append(nc)
            clusters.append(cluster)
        return clusters

    def is_goal_cell_safe(self, wx, wy):
        msg = self.map_msg; gx, gy = self.world_to_grid(wx, wy, msg)
        width, height = msg.info.width, msg.info.height; data = list(msg.data)
        if not self.inside(gx, gy, width, height): return False
        radius = max(2, int(self.goal_safety_radius_m / msg.info.resolution))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = gx + dx, gy + dy
                if self.inside(nx, ny, width, height) and self.is_occupied(data, nx, ny, width):
                    return False
        return True

    def has_unknown_neighbor(self, data, x, y, width, height):
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = x + dx, y + dy
            if self.inside(nx, ny, width, height) and self.is_unknown(data, nx, ny, width):
                return True
        return False

    def is_free(self, data, x, y, width): return 0 <= data[y * width + x] <= self.free_threshold
    def is_unknown(self, data, x, y, width): return data[y * width + x] == self.unknown_value
    def is_occupied(self, data, x, y, width): return data[y * width + x] >= self.occupied_threshold
    def inside(self, x, y, width, height): return 0 <= x < width and 0 <= y < height

    def grid_to_world(self, gx, gy, msg):
        res = msg.info.resolution; ox = msg.info.origin.position.x; oy = msg.info.origin.position.y
        q = msg.info.origin.orientation; yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        lx = (gx + 0.5) * res; ly = (gy + 0.5) * res
        return ox + math.cos(yaw)*lx - math.sin(yaw)*ly, oy + math.sin(yaw)*lx + math.cos(yaw)*ly

    def world_to_grid(self, wx, wy, msg):
        res = msg.info.resolution; ox = msg.info.origin.position.x; oy = msg.info.origin.position.y
        q = msg.info.origin.orientation; yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        dx, dy = wx - ox, wy - oy
        lx = math.cos(-yaw)*dx - math.sin(-yaw)*dy
        ly = math.sin(-yaw)*dx + math.cos(-yaw)*dy
        return int(lx / res), int(ly / res)

    def point_in_allowed_polygon(self, x, y):
        # 如果没有设置 polygon，则默认不限制
        if len(self.allowed_polygon) < 3:
            return True

        inside = False
        pts = self.allowed_polygon
        n = len(pts)
        j = n - 1

        for i in range(n):
            xi, yi = pts[i]
            xj, yj = pts[j]

            intersect = ((yi > y) != (yj > y)) and \
                        (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi)

            if intersect:
                inside = not inside

            j = i

        return inside

    def is_blacklisted(self, x, y):
        return any(math.hypot(x-bx, y-by) < self.blacklist_radius for bx, by in self.blacklist)

    def send_nav_goal(self, goal_xy, robot_pose):
        rx, ry, _ = robot_pose; gx, gy = goal_xy; yaw = math.atan2(gy - ry, gx - rx)
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx); goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(yaw/2.0); goal.pose.pose.orientation.w = math.cos(yaw/2.0)
        self.goal_active = True; self.goal_sent_time = time.time(); self.last_goal_time = time.time(); self.current_goal_xy = (gx, gy)
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_response_cb)
        self.get_logger().info(f'send frontier NavigateToPose goal: x={gx:.2f}, y={gy:.2f}, yaw={yaw:.2f}')
        self.publish_status('goal_sent')

    def goal_response_cb(self, future):
        try: goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'frontier goal send failed: {repr(e)}'); self.goal_active = False; return
        if not goal_handle.accepted:
            self.get_logger().warn('frontier goal rejected by Nav2')
            if self.current_goal_xy is not None: self.blacklist.append(self.current_goal_xy)
            self.goal_active = False; return
        self.goal_handle = goal_handle; self.get_logger().info('frontier goal accepted by Nav2'); self.publish_status('goal_accepted')
        goal_handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        try:
            result = future.result(); status = result.status
            self.get_logger().info(f'frontier goal finished, status={status}')
            if status != GoalStatus.STATUS_SUCCEEDED and self.current_goal_xy is not None:
                self.blacklist.append(self.current_goal_xy)
        except Exception as e:
            self.get_logger().warn(f'frontier goal result error: {repr(e)}')
            if self.current_goal_xy is not None: self.blacklist.append(self.current_goal_xy)
        self.goal_active = False; self.goal_handle = None; self.current_goal_xy = None

    def cancel_goal(self):
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
        self.goal_active = False; self.goal_handle = None; self.current_goal_xy = None

    def quat_to_yaw(self, x, y, z, w):
        return math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
    def normalize_angle(self, a):
        while a > math.pi: a -= 2.0*math.pi
        while a < -math.pi: a += 2.0*math.pi
        return a


def main(args=None):
    rclpy.init(args=args); node = FrontierExplorerNode()
    try: rclpy.spin(node)
    finally:
        node.cancel_goal(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
