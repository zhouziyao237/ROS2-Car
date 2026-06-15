#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import csv
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def get_xy(d, key):
    v = d.get(key) or {}
    if isinstance(v, dict):
        return float(v.get('x', 0.0)), float(v.get('y', 0.0)), float(v.get('z', 0.0))
    return 0.0, 0.0, 0.0


class ErrorWatch(Node):
    def __init__(self, topic, csv_path=''):
        super().__init__('oak_servo_error_watch')
        self.topic = topic
        self.csv_path = csv_path
        self.csv_file = None
        self.writer = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow([
                'wall_time', 'stage', 'status',
                'p_oak_x', 'p_oak_y', 'e_oak_x', 'e_oak_y',
                'e_base_x', 'e_base_y', 'yaw_error',
                'cmd_x', 'cmd_y', 'cmd_wz', 'reason', 'safety'
            ])
        self.sub = self.create_subscription(String, topic, self.cb, 10)
        self.get_logger().info(f'watching {topic}')

    def cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            print(msg.data)
            return
        p = get_xy(d, 'p_oak')
        eo = get_xy(d, 'e_oak')
        eb = get_xy(d, 'e_base')
        cmd = get_xy(d, 'cmd_safe')
        line = (
            f"[{time.strftime('%H:%M:%S')}] "
            f"stage={d.get('stage',''):>8} status={d.get('status',''):>9} "
            f"p_oak=({p[0]:+.3f},{p[1]:+.3f})m "
            f"err_oak=({eo[0]:+.3f},{eo[1]:+.3f})m "
            f"err_base=({eb[0]:+.3f},{eb[1]:+.3f})m "
            f"yaw={float(d.get('yaw_error', 0.0)):+.3f}rad "
            f"cmd=({cmd[0]:+.3f},{cmd[1]:+.3f},{cmd[2]:+.3f}) "
            f"reason={d.get('reason','')} safety={d.get('safety',[])}"
        )
        print(line, flush=True)
        if self.writer:
            self.writer.writerow([
                time.time(), d.get('stage', ''), d.get('status', ''),
                p[0], p[1], eo[0], eo[1], eb[0], eb[1], float(d.get('yaw_error', 0.0)),
                cmd[0], cmd[1], cmd[2], d.get('reason', ''), json.dumps(d.get('safety', []), ensure_ascii=False)
            ])
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Print OAK package servo error/status logs.')
    parser.add_argument('--topic', default='/target_detection/package_servo/status')
    parser.add_argument('--csv', default='', help='Optional CSV path to save error log.')
    known, ros_args = parser.parse_known_args(args=sys.argv[1:])
    rclpy.init(args=ros_args)
    node = ErrorWatch(known.topic, known.csv)
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
