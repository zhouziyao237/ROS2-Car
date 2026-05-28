#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Semantic registrar for final row-column shelf logic.

Long-term semantic library stores shelves only.
A shelf is registered from:
  column_qr: S01-C1
  layer_qr:  S01-L1 / S01-L2 / ...
Package QR is ignored for long-term registration.
"""

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_logistics_interfaces.msg import SemanticObservation
import tf2_ros


class SemanticRegistrarNode(Node):
    def __init__(self):
        super().__init__('semantic_registrar_node')

        self.declare_parameter('semantic_map_path', '/host-home/maps/semantic_library.json')
        self.declare_parameter('input_topic', '/vision/semantic_observation')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('tf_timeout_sec', 0.5)

        self.declare_parameter('shelf_width', 1.85)
        self.declare_parameter('shelf_depth', 0.35)
        self.declare_parameter('shelf_height', 1.50)
        self.declare_parameter('approach_distance', 0.90)
        self.declare_parameter('default_rows', 3)

        self.semantic_map_path = Path(self.get_parameter('semantic_map_path').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        self.shelf_width = float(self.get_parameter('shelf_width').value)
        self.shelf_depth = float(self.get_parameter('shelf_depth').value)
        self.shelf_height = float(self.get_parameter('shelf_height').value)
        self.approach_distance = float(self.get_parameter('approach_distance').value)
        self.default_rows = int(self.get_parameter('default_rows').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(SemanticObservation, self.input_topic, self.obs_cb, 30)

        self.get_logger().info('semantic_registrar_node started: column+layer shelf mode')
        self.get_logger().info(f'input_topic={self.input_topic}')
        self.get_logger().info(f'semantic_map_path={self.semantic_map_path}')

    def obs_cb(self, obs: SemanticObservation):
        if obs.obs_type not in ['column_qr', 'layer_qr']:
            return
        if not obs.shelf_id:
            return
        if not obs.has_pose:
            self.get_logger().warn(f'ignore {obs.obs_type}: has_pose=false')
            return

        pose_map = self.transform_pose_to_map(obs.pose)
        if pose_map is None:
            return

        x = float(pose_map.pose.position.x)
        y = float(pose_map.pose.position.y)
        yaw = self.quat_to_yaw(
            pose_map.pose.orientation.x,
            pose_map.pose.orientation.y,
            pose_map.pose.orientation.z,
            pose_map.pose.orientation.w,
        )
        nx = math.cos(yaw)
        ny = math.sin(yaw)
        nx, ny = self.ensure_normal_faces_robot(x, y, nx, ny)
        yaw = math.atan2(ny, nx)

        now = float(self.get_clock().now().nanoseconds / 1e9)
        data = self.load_library()
        shelf = self.ensure_shelf(data, obs.shelf_id)

        if obs.obs_type == 'column_qr':
            self.update_column_anchor(shelf, obs, x, y, nx, ny, yaw, now)
        elif obs.obs_type == 'layer_qr':
            self.update_layer_anchor(shelf, obs, x, y, nx, ny, yaw, now)

        self.try_fuse_shelf_pose(shelf, now)
        shelf['last_seen'] = now
        shelf['status'] = 'active'
        shelf['confidence'] = max(float(shelf.get('confidence', 0.0)), float(obs.confidence))

        self.save_library(data)

        pose_text = 'invalid'
        source = ''
        if shelf.get('current_map_pose', {}).get('valid', False):
            p = shelf['current_map_pose']
            pose_text = f"({p['x']:.2f},{p['y']:.2f},{p['yaw']:.2f})"
            source = p.get('source', '')

        self.get_logger().info(
            f'updated shelf semantic shelf={obs.shelf_id} obs={obs.obs_type} '
            f'column={obs.column_id} layer={obs.layer_id} qr=({x:.2f},{y:.2f}) '
            f'normal=({nx:.2f},{ny:.2f}) shelf_pose={pose_text} source={source}'
        )

    def update_column_anchor(self, shelf, obs, x, y, nx, ny, yaw, now):
        col = int(obs.col)
        column_id = obs.column_id if obs.column_id else f'C{col}'
        shelf['column_id'] = column_id
        shelf['col'] = col
        shelf['column_anchor'] = {
            'seen': True,
            'column_id': column_id,
            'col': col,
            'x': x,
            'y': y,
            'normal_x': nx,
            'normal_y': ny,
            'yaw': yaw,
            'stamp': now,
            'confidence': float(obs.confidence),
        }

    def update_layer_anchor(self, shelf, obs, x, y, nx, ny, yaw, now):
        row = int(obs.row)
        layer_id = obs.layer_id if obs.layer_id else f'L{row}'
        layers = shelf.setdefault('layers', {})
        layers[layer_id] = {
            'seen': True,
            'layer_id': layer_id,
            'row': row,
            'x': x,
            'y': y,
            'normal_x': nx,
            'normal_y': ny,
            'yaw': yaw,
            'stamp': now,
            'confidence': float(obs.confidence),
        }

    def try_fuse_shelf_pose(self, shelf, now):
        """
        用列二维码 + 行/层二维码实时融合修正货架位姿。

        约定：
          column_qr = 侧面二维码，例如 S03-C3
          layer_qr  = 正面二维码，例如 S03-L1 / S03-L2 / S03-L3

        column_qr 提供侧面约束：
          货架中心 = side_qr_point - side_normal * shelf_width/2

        layer_qr 提供正面约束：
          货架中心 = front_qr_point - front_normal * shelf_depth/2

        yaw / approach_pose 优先用正面 layer_qr 的 normal。
        """
        column_anchor = shelf.get('column_anchor')
        layers = shelf.get('layers', {})

        have_column = column_anchor is not None and bool(column_anchor.get('seen', True))
        have_layers = bool(layers)

        if not have_column and not have_layers:
            return False

        import math

        def norm2(nx, ny):
            n = math.hypot(nx, ny)
            if n < 1e-6:
                return -1.0, 0.0
            return nx / n, ny / n

        centers = []
        front_normal = None
        side_normal = None

        # 1. 正面行二维码：融合多个 layer，得到正面点和正面法向
        if have_layers:
            valid_layers = list(layers.values())
            lx = sum(float(v['x']) for v in valid_layers) / len(valid_layers)
            ly = sum(float(v['y']) for v in valid_layers) / len(valid_layers)
            lnx = sum(float(v.get('normal_x', -1.0)) for v in valid_layers) / len(valid_layers)
            lny = sum(float(v.get('normal_y', 0.0)) for v in valid_layers) / len(valid_layers)
            lnx, lny = norm2(lnx, lny)

            # layer_qr 在正面：中心在正面法向反方向 depth/2
            cx_front = lx - lnx * (self.shelf_depth / 2.0)
            cy_front = ly - lny * (self.shelf_depth / 2.0)

            centers.append((cx_front, cy_front, 0.70))
            front_normal = (lnx, lny)

        # 2. 侧面列二维码：得到侧面点和侧面法向
        if have_column:
            sx = float(column_anchor['x'])
            sy = float(column_anchor['y'])
            snx = float(column_anchor.get('normal_x', -1.0))
            sny = float(column_anchor.get('normal_y', 0.0))
            snx, sny = norm2(snx, sny)

            # column_qr 在侧面：中心在侧面法向反方向 width/2
            cx_side = sx - snx * (self.shelf_width / 2.0)
            cy_side = sy - sny * (self.shelf_width / 2.0)

            centers.append((cx_side, cy_side, 0.30))
            side_normal = (snx, sny)

        # 3. 加权融合中心
        wsum = sum(c[2] for c in centers)
        cx = sum(c[0] * c[2] for c in centers) / wsum
        cy = sum(c[1] * c[2] for c in centers) / wsum

        # 4. 计算 yaw：优先用正面 layer_qr 的 normal
        if front_normal is not None:
            fnx, fny = front_normal
            yaw = math.atan2(fny, fnx)
            source = 'column_layer_qr_depth_fusion' if have_column else 'layer_qr_depth_only'
        elif side_normal is not None:
            snx, sny = side_normal
            # 侧面法向转成正面法向。
            # 如果 RViz 显示方向反了，把 + math.pi/2 改成 - math.pi/2。
            yaw = math.atan2(sny, snx) + math.pi / 2.0
            source = 'column_qr_depth_only'
        else:
            yaw = 0.0
            source = 'unknown_qr_depth'

        # 5. 平滑更新，避免 marker 抖动
        old = shelf.get('current_map_pose', {})
        if old.get('valid', False):
            alpha = 0.35
            old_x = float(old.get('x', cx))
            old_y = float(old.get('y', cy))
            old_yaw = float(old.get('yaw', yaw))

            cx = old_x * (1.0 - alpha) + cx * alpha
            cy = old_y * (1.0 - alpha) + cy * alpha

            dyaw = yaw - old_yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            yaw = old_yaw + dyaw * alpha

        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi

        # 6. approach pose：在正面法向方向外侧停靠，并朝向货架中心
        fx = math.cos(yaw)
        fy = math.sin(yaw)

        approach_x = cx + fx * self.approach_distance
        approach_y = cy + fy * self.approach_distance
        approach_yaw = math.atan2(cy - approach_y, cx - approach_x)

        shelf['current_map_pose'] = {
            'valid': True,
            'x': float(cx),
            'y': float(cy),
            'yaw': float(yaw),
            'stamp': now,
            'source': source,
            'fusion': {
                'has_column_qr': bool(have_column),
                'has_layer_qr': bool(have_layers),
            }
        }

        shelf['approach_pose'] = {
            'valid': True,
            'x': float(approach_x),
            'y': float(approach_y),
            'yaw': float(approach_yaw),
            'distance': float(self.approach_distance),
            'source': source,
        }

        shelf['geometry'] = {
            'width': float(self.shelf_width),
            'depth': float(self.shelf_depth),
            'height': float(self.shelf_height),
            'source': 'nominal_size_column_layer_qr_depth_fusion',
        }

        return True


    def ensure_normal_faces_robot(self, x, y, nx, ny):
        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            return nx, ny
        rx, ry, _ = robot_pose
        vx = rx - x
        vy = ry - y
        if nx * vx + ny * vy < 0.0:
            nx, ny = -nx, -ny
        return nx, ny

    def transform_pose_to_map(self, pose_stamped):
        if not pose_stamped.header.frame_id:
            self.get_logger().warn('pose header.frame_id empty')
            return None
        if pose_stamped.header.frame_id == self.map_frame:
            return pose_stamped
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
            return self.manual_transform_pose_stamped(pose_stamped, tf)
        except Exception as exc:
            self.get_logger().warn(f'failed TF {pose_stamped.header.frame_id}->{self.map_frame}: {repr(exc)}')
            return None

    def manual_transform_pose_stamped(self, pose_stamped, tf):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = tf.header.frame_id

        tq = tf.transform.rotation
        q_tf = (tq.x, tq.y, tq.z, tq.w)
        px = pose_stamped.pose.position.x
        py = pose_stamped.pose.position.y
        pz = pose_stamped.pose.position.z
        rx, ry, rz = self.rotate_vector_by_quat((px, py, pz), q_tf)
        out.pose.position.x = tf.transform.translation.x + rx
        out.pose.position.y = tf.transform.translation.y + ry
        out.pose.position.z = tf.transform.translation.z + rz

        pq = pose_stamped.pose.orientation
        q_pose = (pq.x, pq.y, pq.z, pq.w)
        q_out = self.quat_multiply(q_tf, q_pose)
        out.pose.orientation.x = q_out[0]
        out.pose.orientation.y = q_out[1]
        out.pose.orientation.z = q_out[2]
        out.pose.orientation.w = q_out[3]
        return out

    def lookup_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            q = tf.transform.rotation
            yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
            return float(tf.transform.translation.x), float(tf.transform.translation.y), yaw
        except Exception:
            return None

    def ensure_shelf(self, data, shelf_id):
        shelves = data.setdefault('shelves', {})
        if shelf_id not in shelves:
            shelves[shelf_id] = {
                'shelf_id': shelf_id,
                'column_id': '',
                'col': 0,
                'column_anchor': None,
                'layers': {},
                'current_map_pose': {'valid': False},
                'approach_pose': {'valid': False},
                'geometry': {
                    'width': self.shelf_width,
                    'depth': self.shelf_depth,
                    'height': self.shelf_height,
                },
                'confidence': 0.0,
                'status': 'unknown',
            }
        return shelves[shelf_id]

    def load_library(self):
        if not self.semantic_map_path.exists():
            return self.empty_library()
        try:
            text = self.semantic_map_path.read_text().strip()
            if not text:
                return self.empty_library()
            data = json.loads(text)
            if not isinstance(data, dict):
                return self.empty_library()
            data.setdefault('version', 2)
            data.setdefault('semantic_mode', 'shelf_column_layer')
            data.setdefault('shelves', {})
            return data
        except Exception as exc:
            self.get_logger().warn(f'failed to load semantic library: {repr(exc)}')
            return self.empty_library()

    def empty_library(self):
        return {
            'version': 2,
            'semantic_mode': 'shelf_column_layer',
            'shelves': {},
        }

    def save_library(self, data):
        self.semantic_map_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.semantic_map_path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        tmp.replace(self.semantic_map_path)

    def quat_to_yaw(self, x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def quat_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def quat_conjugate(self, q):
        x, y, z, w = q
        return (-x, -y, -z, w)

    def rotate_vector_by_quat(self, v, q):
        qv = (v[0], v[1], v[2], 0.0)
        qr = self.quat_multiply(self.quat_multiply(q, qv), self.quat_conjugate(q))
        return qr[0], qr[1], qr[2]


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRegistrarNode()
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
