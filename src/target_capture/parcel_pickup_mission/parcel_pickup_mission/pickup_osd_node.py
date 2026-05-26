#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


class PickupOsdNode(Node):
    def __init__(self):
        super().__init__('pickup_osd_node')

        self.declare_parameter('input_image_topic', '/target_position_overlay/image')
        self.declare_parameter('output_image_topic', '/pickup_osd/image')

        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('mission_debug_topic', '/mission/debug')

        self.declare_parameter('qr_state_topic', '/target_matcher/state')
        self.declare_parameter('qr_raw_topic', '/target_matcher/raw')
        self.declare_parameter('target_pose_topic', '/target_pose_base')

        self.declare_parameter('draw_debug_line', True)
        self.declare_parameter('max_debug_len', 80)

        self.bridge = CvBridge()

        self.mission_state = 'IDLE'
        self.mission_debug = ''
        self.qr_state = 'UNKNOWN'
        self.qr_raw = ''
        self.pose = None

        self.image_pub = self.create_publisher(
            Image,
            self.get_parameter('output_image_topic').value,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('input_image_topic').value,
            self.image_cb,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter('mission_state_topic').value,
            self.mission_state_cb,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter('mission_debug_topic').value,
            self.mission_debug_cb,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter('qr_state_topic').value,
            self.qr_state_cb,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter('qr_raw_topic').value,
            self.qr_raw_cb,
            10
        )

        self.create_subscription(
            PointStamped,
            self.get_parameter('target_pose_topic').value,
            self.pose_cb,
            10
        )

        self.get_logger().info('pickup_osd_node started')
        self.get_logger().info(f'input={self.get_parameter("input_image_topic").value}')
        self.get_logger().info(f'output={self.get_parameter("output_image_topic").value}')

    def mission_state_cb(self, msg):
        self.mission_state = str(msg.data)

    def mission_debug_cb(self, msg):
        self.mission_debug = str(msg.data)

    def qr_state_cb(self, msg):
        self.qr_state = str(msg.data)

    def qr_raw_cb(self, msg):
        self.qr_raw = str(msg.data)

    def pose_cb(self, msg):
        self.pose = msg

    def state_to_mode(self, state):
        s = str(state)

        if s in ['IDLE']:
            return 'WAIT_TASK'

        if s in ['PARSE_PICKUP_CODE']:
            return 'PARSE_CODE'

        if s in ['SET_SHELF_COL_TARGET', 'SEARCH_SHELF_COL_QR']:
            return 'SEARCH_SHELF_COL'

        if s in ['SET_SHELF_ROW_TARGET', 'SEARCH_SHELF_ROW_QR']:
            return 'SEARCH_SHELF_ROW'

        if s in ['MOVE_LIFT_TO_ROW_HEIGHT', 'WAIT_ROW_HEIGHT']:
            return 'COARSE_LIFT'

        if s in ['SET_PACKAGE_TARGET', 'SEARCH_PACKAGE_QR']:
            return 'SEARCH_PACKAGE'

        if s in ['CHASSIS_FINE_ALIGN']:
            return 'CHASSIS_X_ALIGN'

        if s in ['LIFT_FINE_ALIGN']:
            return 'LIFT_Y_ALIGN'

        if s in ['FREEZE_LIFT']:
            return 'FREEZE_LIFT'

        if s in ['EXTEND_PLATFORM_AND_HAND', 'WAIT_EXTENDED']:
            return 'EXTEND_PICKUP'

        if s in ['RETRACT_HAND_AND_PLATFORM', 'WAIT_RETRACTED']:
            return 'RETRACT'

        if s in ['DONE']:
            return 'DONE'

        if s in ['ERROR']:
            return 'ERROR'

        return s

    def mode_color(self, mode):
        if mode == 'ERROR':
            return (0, 0, 255)

        if mode in ['DONE']:
            return (0, 255, 0)

        if mode in ['CHASSIS_X_ALIGN', 'LIFT_Y_ALIGN', 'EXTEND_PICKUP']:
            return (0, 255, 255)

        if mode in ['SEARCH_SHELF_COL', 'SEARCH_SHELF_ROW', 'SEARCH_PACKAGE']:
            return (255, 180, 0)

        return (255, 255, 255)

    def put_text(self, img, text, x, y, color=(255, 255, 255), scale=0.55, thickness=1):
        cv2.putText(
            img,
            str(text),
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA
        )

    def draw_osd(self, img):
        h, w = img.shape[:2]

        mode = self.state_to_mode(self.mission_state)
        color = self.mode_color(mode)

        # 半透明黑底
        overlay = img.copy()
        panel_w = min(760, w - 20)
        panel_h = 190
        cv2.rectangle(overlay, (10, 10), (10 + panel_w, 10 + panel_h), (0, 0, 0), -1)
        img[:] = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)

        y = 38

        self.put_text(
            img,
            f'MODE: {mode}',
            24,
            y,
            color=color,
            scale=0.75,
            thickness=2
        )
        y += 30

        self.put_text(
            img,
            f'MISSION_STATE: {self.mission_state}',
            24,
            y,
            color=(255, 255, 255),
            scale=0.55,
            thickness=1
        )
        y += 24

        qr_color = (0, 255, 0) if self.qr_state == 'QR_TRACKING' else (0, 165, 255)
        self.put_text(
            img,
            f'QR_STATE: {self.qr_state}    RAW: {self.qr_raw}',
            24,
            y,
            color=qr_color,
            scale=0.55,
            thickness=1
        )
        y += 24

        if self.pose is not None:
            x_mm = self.pose.point.x * 1000.0
            y_mm = self.pose.point.y * 1000.0
            z_mm = self.pose.point.z * 1000.0

            self.put_text(
                img,
                f'POSE: x={x_mm:+.1f}mm  y={y_mm:+.1f}mm  z={z_mm:.1f}mm',
                24,
                y,
                color=(255, 255, 255),
                scale=0.55,
                thickness=1
            )
        else:
            self.put_text(
                img,
                'POSE: no target pose',
                24,
                y,
                color=(0, 0, 255),
                scale=0.55,
                thickness=1
            )

        y += 24

        control_hint = self.get_control_hint(mode)
        self.put_text(
            img,
            f'CTRL: {control_hint}',
            24,
            y,
            color=(200, 255, 200),
            scale=0.55,
            thickness=1
        )

        y += 24

        if bool(self.get_parameter('draw_debug_line').value):
            max_len = int(self.get_parameter('max_debug_len').value)
            dbg = self.mission_debug[-max_len:]
            self.put_text(
                img,
                f'DEBUG: {dbg}',
                24,
                y,
                color=(180, 180, 180),
                scale=0.45,
                thickness=1
            )

        # 右上角简易状态灯
        cx = w - 40
        cy = 35
        cv2.circle(img, (cx, cy), 14, color, -1)
        cv2.circle(img, (cx, cy), 14, (255, 255, 255), 2)

        return img

    def get_control_hint(self, mode):
        if mode == 'WAIT_TASK':
            return 'waiting /mission/pickup_code'

        if mode == 'SEARCH_SHELF_COL':
            return 'QR target = COL:x'

        if mode == 'SEARCH_SHELF_ROW':
            return 'QR target = ROW:y'

        if mode == 'COARSE_LIFT':
            return 'publish /qr_lift_control/set_lift_mm'

        if mode == 'SEARCH_PACKAGE':
            return 'QR target = package code'

        if mode == 'CHASSIS_X_ALIGN':
            return 'enable chassis align, use target_pose.x'

        if mode == 'LIFT_Y_ALIGN':
            return 'enable lift align, use target_pose.y'

        if mode == 'FREEZE_LIFT':
            return 'freeze lift before extension'

        if mode == 'EXTEND_PICKUP':
            return 'platform first, then hand'

        if mode == 'RETRACT':
            return 'hand retract first, platform retract later'

        if mode == 'DONE':
            return 'mission complete'

        if mode == 'ERROR':
            return 'mission error, check /mission/debug'

        return 'running'

    def image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'failed to convert image: {e}')
            return

        img = self.draw_osd(img)

        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickupOsdNode()

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
