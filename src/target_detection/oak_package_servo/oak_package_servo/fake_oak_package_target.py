#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FakeOakPackageTarget(Node):
    def __init__(self):
        super().__init__('fake_oak_package_target')
        self.declare_parameter('datamatrix_topic', '/oak/datamatrix_3d_result_json')
        self.declare_parameter('request_topic', '/target_detection/package_servo/request')
        self.declare_parameter('package_code', '0301')
        self.declare_parameter('shelf_id', 'S03')
        self.declare_parameter('stage', 'approach')
        self.declare_parameter('camera_x_mm', -120.0)  # negative = package is left in image
        self.declare_parameter('camera_y_mm', 0.0)
        self.declare_parameter('camera_z_mm', 900.0)
        self.declare_parameter('shelf_front_yaw', 0.0)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('send_request', True)
        self.declare_parameter('request_delay_sec', 1.0)

        self.dm_pub = self.create_publisher(String, str(self.get_parameter('datamatrix_topic').value), 10)
        self.req_pub = self.create_publisher(String, str(self.get_parameter('request_topic').value), 10)
        self.start = time.time()
        self.request_sent = False
        hz = float(self.get_parameter('publish_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, hz), self.on_timer)
        self.get_logger().info('fake_oak_package_target started')

    def on_timer(self):
        code = str(self.get_parameter('package_code').value)
        xyz = {
            'x': float(self.get_parameter('camera_x_mm').value),
            'y': float(self.get_parameter('camera_y_mm').value),
            'z': float(self.get_parameter('camera_z_mm').value),
        }
        payload = {
            'timestamp': time.time(),
            'count': 1,
            'codes': [code],
            'items': [{
                'code': code,
                'format': 'DataMatrix',
                'center_px': [960, 540],
                'valid_depth': True,
                'camera_xyz_mm': xyz,
            }],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.dm_pub.publish(msg)

        if bool(self.get_parameter('send_request').value) and not self.request_sent:
            if time.time() - self.start >= float(self.get_parameter('request_delay_sec').value):
                req = {
                    'command': str(self.get_parameter('stage').value),
                    'package_code': code,
                    'shelf_id': str(self.get_parameter('shelf_id').value),
                    'shelf_front_yaw': float(self.get_parameter('shelf_front_yaw').value),
                    'timeout_sec': 8.0,
                    'request_id': f'fake_{int(time.time())}',
                }
                m = String()
                m.data = json.dumps(req)
                self.req_pub.publish(m)
                self.request_sent = True
                self.get_logger().info(f'sent fake request: {m.data}')


def main(args=None):
    rclpy.init(args=args)
    node = FakeOakPackageTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
