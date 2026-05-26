#!/usr/bin/env python3
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String, Float32
from rosidl_runtime_py.utilities import get_message


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ArmState(str, Enum):
    IDLE = "IDLE"
    WAIT_DEPTH_STABLE = "WAIT_DEPTH_STABLE"
    EXTEND_PLATFORM_BY_DEPTH = "EXTEND_PLATFORM_BY_DEPTH"
    WAIT_DEPTH_AFTER_PLATFORM = "WAIT_DEPTH_AFTER_PLATFORM"
    COMPUTE_HAND_TARGET = "COMPUTE_HAND_TARGET"
    EXTEND_HAND = "EXTEND_HAND"
    HOLD_EXTENDED = "HOLD_EXTENDED"
    RETRACT_HAND = "RETRACT_HAND"
    RETRACT_PLATFORM = "RETRACT_PLATFORM"


class QrLiftControlNode(Node):
    def __init__(self):
        super().__init__('qr_lift_control_node')

        # =========================
        # Topics
        # =========================
        self.declare_parameter('target_pose_topic', '/target_pose_base')
        self.declare_parameter('platform_target_topic', '/platform/target')
        self.declare_parameter('platform_target_type', 'auto_aim_interfaces/msg/PlatformTarget')

        self.declare_parameter('enable_topic', '/qr_lift_control/enable')
        self.declare_parameter('freeze_topic', '/qr_lift_control/freeze')
        self.declare_parameter('home_topic', '/qr_lift_control/home')
        self.declare_parameter('set_lift_topic', '/qr_lift_control/set_lift_mm')
        self.declare_parameter('debug_topic', '/qr_lift_control/debug')

        self.declare_parameter('arm_start_topic', '/pickup_extend/start')
        self.declare_parameter('arm_retract_topic', '/pickup_extend/retract')

        # =========================
        # PlatformTarget fields
        # =========================
        self.declare_parameter('lift_field', 'target_lift_z_mm')
        self.declare_parameter('platform_extend_field', 'target_slide_x_mm')
        self.declare_parameter('hand_extend_field', 'target_screw_x_mm')

        # =========================
        # Basic
        # =========================
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('verbose', True)

        # =========================
        # Lift control
        # x = 左右
        # y = 升降
        # z = 深度
        # =========================
        self.declare_parameter('platform_min_height_mm', 0.0)
        self.declare_parameter('platform_max_height_mm', 330.0)
        self.declare_parameter('initial_lift_target_mm', 0.0)

        self.declare_parameter('lift_axis', 'y')
        self.declare_parameter('lift_axis_target_mm', 0.0)

        self.declare_parameter('lift_step_mm', 0.5)
        self.declare_parameter('lift_deadband_mm', 30.0)
        self.declare_parameter('lift_direction', 1.0)

        self.declare_parameter('lift_filter_window', 9)
        self.declare_parameter('max_jump_lift_mm', 60.0)
        self.declare_parameter('lift_required_same_direction_frames', 5)
        self.declare_parameter('lift_command_interval_sec', 0.20)

        self.declare_parameter('lost_timeout_sec', 0.60)
        self.declare_parameter('return_home_when_lost', False)
        self.declare_parameter('return_home_step_mm', 4.0)

        # =========================
        # First stage: platform arm
        # 深度相机跟着第一节平台走
        # =========================
        self.declare_parameter('arm_depth_axis', 'z')
        self.declare_parameter('arm_depth_direction', 1.0)

        self.declare_parameter('platform_extend_min_mm', 0.0)
        self.declare_parameter('platform_extend_max_mm', 120.0)
        self.declare_parameter('platform_extend_step_mm', 5.0)

        # 第一节平台伸到这个 z 附近就停止。
        # 例如：平台伸出后，希望相机距离包裹仍有 220mm。
        self.declare_parameter('platform_z_target_mm', 220.0)
        self.declare_parameter('platform_z_deadband_mm', 20.0)

        # =========================
        # Second stage: hand arm
        # 第二节小臂伸出时，相机不跟着走
        # =========================
        self.declare_parameter('hand_extend_min_mm', 0.0)
        self.declare_parameter('hand_extend_max_mm', 200.0)

        # 小臂末端最终距离包裹保留的安全距离
        self.declare_parameter('pickup_keep_distance_mm', 50.0)

        # 相机到第二节小臂零位末端的前后偏移
        # 如果小臂零位末端比相机更靠前 30mm，填 30
        self.declare_parameter('camera_to_hand_tip_zero_z_mm', 0.0)

        # 第二节伸出补偿：伸不够加正数，伸太多加负数
        self.declare_parameter('hand_extend_offset_mm', 0.0)

        # 深度稳定判断
        self.declare_parameter('arm_depth_filter_window', 5)
        self.declare_parameter('arm_depth_required_valid_samples', 4)
        self.declare_parameter('arm_depth_max_jump_mm', 80.0)

        # 状态等待时间
        self.declare_parameter('platform_settle_wait_sec', 0.8)
        self.declare_parameter('hand_extend_wait_sec', 1.2)
        self.declare_parameter('hand_retract_wait_sec', 1.0)
        self.declare_parameter('platform_retract_wait_sec', 1.0)

        # =========================
        # Runtime state
        # =========================
        self.enabled = True
        self.freeze = False

        self.lift_cmd_mm = float(self.get_parameter('initial_lift_target_mm').value)
        self.platform_extend_cmd_mm = 0.0
        self.hand_extend_cmd_mm = 0.0

        self.last_target_pose = None
        self.last_target_time = None
        self.last_lift_axis_mm = None
        self.last_arm_depth_mm = None

        lift_win = max(1, int(self.get_parameter('lift_filter_window').value))
        self.lift_axis_buf = deque(maxlen=lift_win)

        arm_win = max(1, int(self.get_parameter('arm_depth_filter_window').value))
        self.arm_depth_buf = deque(maxlen=arm_win)

        self.last_desired_dir = 0
        self.same_direction_count = 0
        self.last_lift_command_time = 0.0

        self.arm_state = ArmState.IDLE
        self.arm_state_enter_time = self.now_sec()

        self.hand_target_mm = 0.0

        platform_type_name = self.get_parameter('platform_target_type').value
        self.PlatformTarget = get_message(platform_type_name)

        self.platform_pub = self.create_publisher(
            self.PlatformTarget,
            self.get_parameter('platform_target_topic').value,
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            self.get_parameter('debug_topic').value,
            10
        )

        self.create_subscription(
            PointStamped,
            self.get_parameter('target_pose_topic').value,
            self.target_pose_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('enable_topic').value,
            self.enable_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('freeze_topic').value,
            self.freeze_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('home_topic').value,
            self.home_cb,
            10
        )

        self.create_subscription(
            Float32,
            self.get_parameter('set_lift_topic').value,
            self.set_lift_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('arm_start_topic').value,
            self.arm_start_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('arm_retract_topic').value,
            self.arm_retract_cb,
            10
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(1.0, rate), self.loop)

        self.log_debug(
            f'qr_lift_control_node started | '
            f'lift={self.lift_cmd_mm:.1f}mm | '
            f'lift_axis={self.get_parameter("lift_axis").value} | '
            f'lift_target={float(self.get_parameter("lift_axis_target_mm").value):.1f}mm'
        )

    # =========================
    # Utilities
    # =========================
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log_debug(self, text):
        if bool(self.get_parameter('verbose').value):
            msg = String()
            msg.data = str(text)
            self.debug_pub.publish(msg)
            self.get_logger().info(str(text))

    def get_axis_mm(self, pose_msg, axis):
        axis = str(axis).lower().strip()

        if axis == 'x':
            return float(pose_msg.point.x) * 1000.0
        if axis == 'y':
            return float(pose_msg.point.y) * 1000.0
        if axis == 'z':
            return float(pose_msg.point.z) * 1000.0

        return float(pose_msg.point.z) * 1000.0

    def has_recent_target(self):
        if self.last_target_time is None:
            return False

        dt = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        return dt <= float(self.get_parameter('lost_timeout_sec').value)

    def median(self, q):
        data = sorted(q)
        return data[len(data) // 2]

    def stable_arm_depth_mm(self):
        required = int(self.get_parameter('arm_depth_required_valid_samples').value)

        if len(self.arm_depth_buf) < required:
            return None

        return self.median(self.arm_depth_buf)

    # =========================
    # Callbacks
    # =========================
    def target_pose_cb(self, msg):
        self.last_target_pose = msg
        self.last_target_time = self.get_clock().now()

        # y 升降数据
        lift_axis = self.get_parameter('lift_axis').value
        current_lift_mm = self.get_axis_mm(msg, lift_axis)

        if self.last_lift_axis_mm is not None:
            max_jump = float(self.get_parameter('max_jump_lift_mm').value)

            if abs(current_lift_mm - self.last_lift_axis_mm) <= max_jump:
                self.last_lift_axis_mm = current_lift_mm
                self.lift_axis_buf.append(current_lift_mm)
            else:
                self.log_debug(
                    f'LIFT_IGNORE_JUMP current={current_lift_mm:.1f}, '
                    f'last={self.last_lift_axis_mm:.1f}'
                )
        else:
            self.last_lift_axis_mm = current_lift_mm
            self.lift_axis_buf.append(current_lift_mm)

        # z 深度数据
        depth_axis = self.get_parameter('arm_depth_axis').value
        depth_mm = self.get_axis_mm(msg, depth_axis)
        depth_mm *= float(self.get_parameter('arm_depth_direction').value)

        if self.last_arm_depth_mm is not None:
            max_jump = float(self.get_parameter('arm_depth_max_jump_mm').value)

            if abs(depth_mm - self.last_arm_depth_mm) <= max_jump:
                self.last_arm_depth_mm = depth_mm
                self.arm_depth_buf.append(depth_mm)
            else:
                self.log_debug(
                    f'DEPTH_IGNORE_JUMP current={depth_mm:.1f}, '
                    f'last={self.last_arm_depth_mm:.1f}'
                )
        else:
            self.last_arm_depth_mm = depth_mm
            self.arm_depth_buf.append(depth_mm)

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)
        self.log_debug(f'enabled={self.enabled}')

    def freeze_cb(self, msg):
        self.freeze = bool(msg.data)
        self.log_debug(f'freeze={self.freeze}')

    def home_cb(self, msg):
        if not bool(msg.data):
            return

        self.freeze = False
        self.lift_cmd_mm = 0.0
        self.platform_extend_cmd_mm = 0.0
        self.hand_extend_cmd_mm = 0.0
        self.hand_target_mm = 0.0

        self.last_desired_dir = 0
        self.same_direction_count = 0

        self.arm_state = ArmState.IDLE
        self.arm_state_enter_time = self.now_sec()

        self.publish_platform_target()
        self.log_debug('HOME all zero')

    def set_lift_cb(self, msg):
        mn = float(self.get_parameter('platform_min_height_mm').value)
        mx = float(self.get_parameter('platform_max_height_mm').value)

        old = self.lift_cmd_mm
        self.lift_cmd_mm = clamp(float(msg.data), mn, mx)

        self.last_desired_dir = 0
        self.same_direction_count = 0

        self.log_debug(
            f'SET_LIFT coarse target {old:.1f}->{self.lift_cmd_mm:.1f}mm'
        )

    def arm_start_cb(self, msg):
        if not bool(msg.data):
            return

        if not self.has_recent_target():
            self.log_debug('ARM_START rejected: no recent target')
            return

        self.arm_depth_buf.clear()

        # 第一节平台先伸，第二节保持 0
        self.platform_extend_cmd_mm = clamp(
            self.platform_extend_cmd_mm,
            float(self.get_parameter('platform_extend_min_mm').value),
            float(self.get_parameter('platform_extend_max_mm').value)
        )
        self.hand_extend_cmd_mm = 0.0
        self.hand_target_mm = 0.0

        self.arm_state = ArmState.WAIT_DEPTH_STABLE
        self.arm_state_enter_time = self.now_sec()

        self.publish_platform_target()
        self.log_debug('ARM_STATE -> WAIT_DEPTH_STABLE')

    def arm_retract_cb(self, msg):
        if not bool(msg.data):
            return

        # 收回顺序：第二节先收，第一节后收
        self.hand_extend_cmd_mm = 0.0
        self.arm_state = ArmState.RETRACT_HAND
        self.arm_state_enter_time = self.now_sec()
        self.publish_platform_target()
        self.log_debug('ARM_STATE -> RETRACT_HAND')

    # =========================
    # Lift y fine alignment
    # =========================
    def update_lift(self):
        if not self.enabled:
            return

        if self.freeze:
            return

        if not self.has_recent_target():
            if bool(self.get_parameter('return_home_when_lost').value):
                step = float(self.get_parameter('return_home_step_mm').value)
                self.lift_cmd_mm = max(0.0, self.lift_cmd_mm - step)

            self.last_desired_dir = 0
            self.same_direction_count = 0
            return

        if len(self.lift_axis_buf) < self.lift_axis_buf.maxlen:
            return

        current_mm = self.median(self.lift_axis_buf)
        target_mm = float(self.get_parameter('lift_axis_target_mm').value)

        err_mm = current_mm - target_mm

        deadband = float(self.get_parameter('lift_deadband_mm').value)
        step = float(self.get_parameter('lift_step_mm').value)
        direction = float(self.get_parameter('lift_direction').value)

        if abs(err_mm) <= deadband:
            desired_dir = 0
        elif err_mm > 0:
            desired_dir = 1
        else:
            desired_dir = -1

        if desired_dir == self.last_desired_dir:
            self.same_direction_count += 1
        else:
            self.last_desired_dir = desired_dir
            self.same_direction_count = 1

        required_frames = max(
            1,
            int(self.get_parameter('lift_required_same_direction_frames').value)
        )

        now = self.now_sec()
        interval = float(self.get_parameter('lift_command_interval_sec').value)

        delta = 0.0

        if desired_dir == 0:
            delta = 0.0
        elif self.same_direction_count < required_frames:
            delta = 0.0
        elif now - self.last_lift_command_time < interval:
            delta = 0.0
        else:
            delta = direction * step * float(desired_dir)
            self.last_lift_command_time = now

        mn = float(self.get_parameter('platform_min_height_mm').value)
        mx = float(self.get_parameter('platform_max_height_mm').value)

        old = self.lift_cmd_mm
        self.lift_cmd_mm = clamp(self.lift_cmd_mm + delta, mn, mx)

        if abs(delta) > 1e-6:
            self.log_debug(
                f'LIFT current={current_mm:.1f} target={target_mm:.1f} '
                f'err={err_mm:.1f} delta={delta:.1f} '
                f'lift={old:.1f}->{self.lift_cmd_mm:.1f}'
            )

    # =========================
    # Arm two-stage extension
    # =========================
    def update_arm_state(self):
        now = self.now_sec()
        elapsed = now - self.arm_state_enter_time

        if self.arm_state == ArmState.IDLE:
            return

        if self.arm_state == ArmState.WAIT_DEPTH_STABLE:
            z = self.stable_arm_depth_mm()

            if z is None:
                return

            self.log_debug(f'ARM depth stable before platform z={z:.1f}mm')
            self.arm_state = ArmState.EXTEND_PLATFORM_BY_DEPTH
            self.arm_state_enter_time = now
            self.log_debug('ARM_STATE -> EXTEND_PLATFORM_BY_DEPTH')
            return

        if self.arm_state == ArmState.EXTEND_PLATFORM_BY_DEPTH:
            z = self.stable_arm_depth_mm()

            if z is None:
                return

            z_target = float(self.get_parameter('platform_z_target_mm').value)
            z_deadband = float(self.get_parameter('platform_z_deadband_mm').value)

            p_min = float(self.get_parameter('platform_extend_min_mm').value)
            p_max = float(self.get_parameter('platform_extend_max_mm').value)
            p_step = float(self.get_parameter('platform_extend_step_mm').value)

            # 第一节平台伸出时，相机跟着走，所以用 z 闭环靠近
            need_continue = z > (z_target + z_deadband)
            not_at_max = self.platform_extend_cmd_mm < p_max

            if need_continue and not_at_max:
                old = self.platform_extend_cmd_mm
                self.platform_extend_cmd_mm = clamp(old + p_step, p_min, p_max)
                self.hand_extend_cmd_mm = 0.0

                self.log_debug(
                    f'PLATFORM_EXTEND z={z:.1f} target={z_target:.1f} '
                    f'platform={old:.1f}->{self.platform_extend_cmd_mm:.1f}'
                )
                return

            self.arm_state = ArmState.WAIT_DEPTH_AFTER_PLATFORM
            self.arm_state_enter_time = now
            self.arm_depth_buf.clear()

            self.log_debug(
                f'PLATFORM_DONE z={z:.1f} platform={self.platform_extend_cmd_mm:.1f}'
            )
            self.log_debug('ARM_STATE -> WAIT_DEPTH_AFTER_PLATFORM')
            return

        if self.arm_state == ArmState.WAIT_DEPTH_AFTER_PLATFORM:
            if elapsed < float(self.get_parameter('platform_settle_wait_sec').value):
                return

            z = self.stable_arm_depth_mm()

            if z is None:
                return

            self.log_debug(f'ARM depth stable after platform z={z:.1f}mm')
            self.arm_state = ArmState.COMPUTE_HAND_TARGET
            self.arm_state_enter_time = now
            return

        if self.arm_state == ArmState.COMPUTE_HAND_TARGET:
            z = self.stable_arm_depth_mm()

            if z is None:
                return

            keep = float(self.get_parameter('pickup_keep_distance_mm').value)
            cam_to_tip = float(self.get_parameter('camera_to_hand_tip_zero_z_mm').value)
            offset = float(self.get_parameter('hand_extend_offset_mm').value)

            h_min = float(self.get_parameter('hand_extend_min_mm').value)
            h_max = float(self.get_parameter('hand_extend_max_mm').value)

            # 第二节伸出时，相机不再跟着走，所以根据第一节停下后的 z 计算第二节目标
            target = z - keep - cam_to_tip + offset
            self.hand_target_mm = clamp(target, h_min, h_max)
            self.hand_extend_cmd_mm = self.hand_target_mm

            self.arm_state = ArmState.EXTEND_HAND
            self.arm_state_enter_time = now

            self.log_debug(
                f'HAND_TARGET z={z:.1f} keep={keep:.1f} cam_to_tip={cam_to_tip:.1f} '
                f'offset={offset:.1f} hand={self.hand_target_mm:.1f}'
            )
            self.log_debug('ARM_STATE -> EXTEND_HAND')
            return

        if self.arm_state == ArmState.EXTEND_HAND:
            if elapsed >= float(self.get_parameter('hand_extend_wait_sec').value):
                self.arm_state = ArmState.HOLD_EXTENDED
                self.arm_state_enter_time = now
                self.log_debug('ARM_STATE -> HOLD_EXTENDED')
            return

        if self.arm_state == ArmState.HOLD_EXTENDED:
            return

        if self.arm_state == ArmState.RETRACT_HAND:
            if elapsed >= float(self.get_parameter('hand_retract_wait_sec').value):
                self.platform_extend_cmd_mm = 0.0
                self.arm_state = ArmState.RETRACT_PLATFORM
                self.arm_state_enter_time = now
                self.log_debug('ARM_STATE -> RETRACT_PLATFORM')
            return

        if self.arm_state == ArmState.RETRACT_PLATFORM:
            if elapsed >= float(self.get_parameter('platform_retract_wait_sec').value):
                self.platform_extend_cmd_mm = 0.0
                self.hand_extend_cmd_mm = 0.0
                self.hand_target_mm = 0.0
                self.arm_state = ArmState.IDLE
                self.arm_state_enter_time = now
                self.log_debug('ARM_STATE -> IDLE')
            return

    # =========================
    # Publish PlatformTarget
    # =========================
    def set_field_if_exists(self, msg, field, value):
        field = str(field).strip()

        if hasattr(msg, field):
            setattr(msg, field, float(value))
            return

        self.get_logger().error(f'PlatformTarget has no field: {field}')

    def publish_platform_target(self):
        msg = self.PlatformTarget()

        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'platform_target'

        self.set_field_if_exists(
            msg,
            self.get_parameter('lift_field').value,
            self.lift_cmd_mm
        )

        self.set_field_if_exists(
            msg,
            self.get_parameter('platform_extend_field').value,
            self.platform_extend_cmd_mm
        )

        self.set_field_if_exists(
            msg,
            self.get_parameter('hand_extend_field').value,
            self.hand_extend_cmd_mm
        )

        self.platform_pub.publish(msg)

    def loop(self):
        self.update_lift()
        self.update_arm_state()
        self.publish_platform_target()


def main(args=None):
    rclpy.init(args=args)
    node = QrLiftControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_platform_target()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()