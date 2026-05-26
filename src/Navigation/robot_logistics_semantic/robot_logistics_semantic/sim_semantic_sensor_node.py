#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_logistics_interfaces.msg import SemanticObservation
import tf2_ros


def quat_from_yaw(yaw):
    import math
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return z, w

class SimSemanticSensorNode(Node):
    """Gazebo-only semantic vision simulator.

    It simulates QR/depth recognition by publishing SemanticObservation when the robot is
    near a shelf. This lets the full semantic-library, relocalization, task and frontier
    pipeline run in Gazebo without depending on real QR image decoding.
    """
    def __init__(self):
        super().__init__('sim_semantic_sensor_node')
        self.declare_parameter('enabled', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('detection_radius', 4.0)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('confidence', 0.92)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.detection_radius = float(self.get_parameter('detection_radius').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.pub = self.create_publisher(SemanticObservation, '/vision/semantic_observation', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose_map', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.shelves = [
            {'id': 'S01', 'x': 3.2, 'y': 1.7, 'yaw': 0.0, 'pkg': 'A1234'},
            {'id': 'S02', 'x': 3.4, 'y': -1.7, 'yaw': 0.0, 'pkg': 'B5678'},
            {'id': 'S03', 'x': 6.4, 'y': 0.0, 'yaw': math.pi, 'pkg': 'C9999'},
        ]
        self.timer = self.create_timer(float(self.get_parameter('publish_period_sec').value), self.timer_cb)
        self.get_logger().info('sim semantic sensor started: publishes /vision/semantic_observation when near shelves')

    def timer_cb(self):
        if not self.enabled:
            return
        robot = self.lookup_robot()
        if robot is None:
            return
        rx, ry, ryaw = robot
        for s in self.shelves:
            d = math.hypot(s['x'] - rx, s['y'] - ry)
            if d > self.detection_radius:
                continue
            # Loose front-facing check to mimic camera FOV, but keep permissive for demo.
            heading = math.atan2(s['y'] - ry, s['x'] - rx)
            if abs(self.norm(heading - ryaw)) > 2.2:
                continue
            self.publish_shelf(s)

    def publish_shelf(self, s):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(s['x'])
        pose.pose.position.y = float(s['y'])
        pose.pose.position.z = 0.0
        z, w = quat_from_yaw(float(s['yaw']))
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.pose_pub.publish(pose)
        sid = s['id']
        # shelf anchor
        self.pub_obs('shelf_anchor', sid, 'front', '', '', '', 0, 0, pose)
        # columns and layers
        for c in range(1, 4):
            self.pub_obs('column_qr', sid, 'front', f'C{c}', '', '', 0, c, pose)
        for r in range(1, 4):
            self.pub_obs('layer_qr', sid, 'front', '', f'L{r}', '', r, 0, pose)
        # sample package in L2-C3
        self.pub_obs('package_qr', sid, 'front', 'C3', 'L2', s['pkg'], 2, 3, pose)
        self.get_logger().info(f'sim detected {sid} and package {s["pkg"]}')

    def pub_obs(self, obs_type, shelf_id, face, column_id, layer_id, package_code, row, col, pose):
        msg = SemanticObservation()
        msg.header = pose.header
        msg.obs_type = obs_type
        msg.shelf_id = shelf_id
        msg.face = face
        msg.column_id = column_id
        msg.layer_id = layer_id
        msg.package_code = package_code
        msg.row = int(row)
        msg.col = int(col)
        msg.pose = pose
        msg.confidence = float(self.confidence)
        msg.has_pose = True
        self.pub.publish(msg)

    def lookup_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return tf.transform.translation.x, tf.transform.translation.y, yaw
        except Exception:
            return None

    def norm(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = SimSemanticSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
