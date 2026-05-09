#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ocr_msg.msg import OcrRequest, OcrRequestArray
from yolo_msg.msg import Box2D


class TestPub(Node):
    def __init__(self):
        super().__init__('test_pub_ocr_request')
        self.pub = self.create_publisher(OcrRequestArray, '/vision/ocr_requests', 10)
        self.timer = self.create_timer(1.0, self.once)
        self.done = False

    def once(self):
        if self.done:
            return
        msg = OcrRequestArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_rgb_optical_frame'

        req = OcrRequest()
        req.header = msg.header
        req.target_id = 'manual_0'
        req.class_name = 'manual'
        req.yolo_confidence = 1.0
        req.need_ocr = True
        req.box = Box2D(x1=0, y1=0, x2=640, y2=480, center_u=320, center_v=240, width=640, height=480)
        msg.requests.append(req)
        self.pub.publish(msg)
        self.get_logger().info('Published test OcrRequestArray.')
        self.done = True


def main():
    rclpy.init()
    node = TestPub()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
