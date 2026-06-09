#!/usr/bin/env python3
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class Checker(Node):
    def __init__(self, args):
        super().__init__('check_oak_mount_yaw_offset')
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def run_once(self):
        try:
            tf_base = self.tf_buffer.lookup_transform(
                self.args.fixed_frame, self.args.base_frame, Time(), timeout=Duration(seconds=1.0)
            )
            tf_oak = self.tf_buffer.lookup_transform(
                self.args.fixed_frame, self.args.oak_frame, Time(), timeout=Duration(seconds=1.0)
            )
            tf_fwd = self.tf_buffer.lookup_transform(
                self.args.fixed_frame, self.args.oak_forward_frame, Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return 1

        base_yaw = quat_to_yaw(tf_base.transform.rotation)
        ox = tf_oak.transform.translation.x
        oy = tf_oak.transform.translation.y
        fx = tf_fwd.transform.translation.x
        fy = tf_fwd.transform.translation.y
        oak_yaw = math.atan2(fy - oy, fx - ox)
        offset = norm(oak_yaw - base_yaw)

        print('==============================================')
        print(f'fixed_frame = {self.args.fixed_frame}')
        print(f'base_yaw    = {base_yaw:.3f} rad = {math.degrees(base_yaw):.1f} deg')
        print(f'oak_yaw     = {oak_yaw:.3f} rad = {math.degrees(oak_yaw):.1f} deg')
        print(f'oak-base offset = {offset:.3f} rad = {math.degrees(offset):.1f} deg')
        print('==============================================')
        return 0


def main():
    parser = argparse.ArgumentParser(description='Check OAK mounting yaw offset from TF.')
    parser.add_argument('--fixed-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--oak-frame', default='oak_camera_link')
    parser.add_argument('--oak-forward-frame', default='oak_camera_forward_link')
    args = parser.parse_args()

    rclpy.init()
    node = Checker(args)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    rc = node.run_once()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
