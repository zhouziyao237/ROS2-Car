#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from auto_aim_interfaces.msg import PlatformTarget, PlatformCurrentStatus


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def parse_xyz_mm(value: Any) -> Optional[Tuple[float, float, float]]:
    if value is None:
        return None
    if isinstance(value, dict) and all(k in value for k in ('x', 'y', 'z')):
        try:
            return float(value['x']), float(value['y']), float(value['z'])
        except Exception:
            return None
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            return float(value[0]), float(value[1]), float(value[2])
        except Exception:
            return None
    return None


def parse_center_px(item: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(item, dict):
        return None
    for key in ('center_px', 'code_center_px', 'pixel_center', 'center'):
        p = item.get(key)
        if isinstance(p, dict):
            u = p.get('u', p.get('x', p.get('cx')))
            v = p.get('v', p.get('y', p.get('cy')))
            if u is not None and v is not None:
                try:
                    return float(u), float(v)
                except Exception:
                    pass
        if isinstance(p, (list, tuple)) and len(p) >= 2:
            try:
                return float(p[0]), float(p[1])
            except Exception:
                pass
    bbox = item.get('bbox') or item.get('box') or item.get('rect')
    if isinstance(bbox, dict):
        try:
            if all(k in bbox for k in ('x', 'y', 'w', 'h')):
                return float(bbox['x']) + float(bbox['w']) * 0.5, float(bbox['y']) + float(bbox['h']) * 0.5
            if all(k in bbox for k in ('xmin', 'ymin', 'xmax', 'ymax')):
                return (float(bbox['xmin']) + float(bbox['xmax'])) * 0.5, (float(bbox['ymin']) + float(bbox['ymax'])) * 0.5
        except Exception:
            pass
    if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
        try:
            return (float(bbox[0]) + float(bbox[2])) * 0.5, (float(bbox[1]) + float(bbox[3])) * 0.5
        except Exception:
            pass
    pts = item.get('corners') or item.get('points') or item.get('polygon')
    if isinstance(pts, list) and pts:
        us, vs = [], []
        for p in pts:
            try:
                if isinstance(p, dict):
                    u = p.get('u', p.get('x'))
                    v = p.get('v', p.get('y'))
                    if u is None or v is None:
                        continue
                    us.append(float(u)); vs.append(float(v))
                elif isinstance(p, (list, tuple)) and len(p) >= 2:
                    us.append(float(p[0])); vs.append(float(p[1]))
            except Exception:
                continue
        if us and vs:
            return sum(us) / len(us), sum(vs) / len(vs)
    return None


def iter_detection_items(payload: Any) -> Iterable[Dict[str, Any]]:
    if isinstance(payload, dict):
        for key in ('items', 'results', 'detections', 'codes'):
            if isinstance(payload.get(key), list):
                for item in payload[key]:
                    if isinstance(item, dict):
                        yield item
        if 'code' in payload:
            yield payload
    elif isinstance(payload, list):
        for item in payload:
            if isinstance(item, dict):
                yield item


class OakGuidedPickupNode(Node):
    def __init__(self) -> None:
        super().__init__('oak_guided_pickup')

        self.declare_parameter('request_topic', '/target_capture/oak_guided_pickup/request')
        self.declare_parameter('status_topic', '/target_capture/oak_guided_pickup/status')
        self.declare_parameter('startup_zero_done_topic', '/target_capture/oak_guided_pickup/startup_zero_done')
        self.declare_parameter('datamatrix_topic', '/oak/datamatrix_3d_result_json')
        self.declare_parameter('platform_current_status_topic', '/platform/current_status')
        self.declare_parameter('platform_target_topic', '/platform/target')
        self.declare_parameter('chassis_control_mode_topic', '/chassis_control_mode')

        # Startup safety zero: in IDLE and auto mode, lower lift target to 0 slowly.
        # This clears stale platform targets without sudden drop.
        self.declare_parameter('enable_startup_safe_zero', True)
        self.declare_parameter('startup_safe_zero_target_mm', 0.0)
        self.declare_parameter('startup_safe_zero_tolerance_mm', 3.0)
        self.declare_parameter('startup_safe_zero_timeout_sec', 120.0)
        self.declare_parameter('startup_safe_zero_require_auto', True)
        self.declare_parameter('startup_safe_zero_confirm_sec', 3.0)
        self.declare_parameter('startup_auto_hold_sec', 1.0)
        self.declare_parameter('startup_done_heartbeat_sec', 0.5)
        self.declare_parameter('startup_zero_follow_actual_jump_mm', 3.0)

        # rm_serial fallback mode source. /chassis_control_mode is primary.
        # /platform/current_status.control_mode is only used as fallback.
        self.declare_parameter('use_platform_control_mode_fallback', True)
        self.declare_parameter('platform_auto_control_mode_value', 1)
        self.declare_parameter('platform_manual_control_mode_value', 0)
        self.declare_parameter('platform_control_mode_as_auto_if_nonzero', True)

        # RGB custom align point for lift. Only replaces image-center V target.
        # It does not modify TF/URDF/depth approach logic.
        self.declare_parameter('enable_custom_align_point', True)
        self.declare_parameter('align_u_px', 960.0)
        self.declare_parameter('align_v_px', 540.0)
        self.declare_parameter('visual_align_point_topic', '/oak/visual_align_point')
        self.declare_parameter('kv_to_lift_mm_per_px', 0.25)
        self.declare_parameter('v_tolerance_px', 20.0)
        self.declare_parameter('lift_pixel_direction', 1.0)
        # Legacy hand-eye parameters are declared only for backward-compatible YAML.
        # They are ignored by the new slider-point lift logic.
        self.declare_parameter('enable_pickup_visual_target', False)
        self.declare_parameter('pickup_ref_oak_x_a', 0.0)
        self.declare_parameter('pickup_ref_oak_x_b', 550.0)
        self.declare_parameter('pickup_ref_oak_y_a', 0.0)
        self.declare_parameter('pickup_ref_oak_y_b', 0.0)
        self.declare_parameter('pickup_ref_oak_z_a', 1.0)
        self.declare_parameter('pickup_ref_oak_z_b', -90.0)
        self.declare_parameter('package_pick_x_offset_mm', 0.0)
        self.declare_parameter('package_pick_y_offset_mm', 0.0)
        self.declare_parameter('package_pick_z_offset_mm', 30.0)

        self.declare_parameter('require_chassis_auto_for_platform_control', True)
        self.declare_parameter('allow_control_when_chassis_mode_unknown', False)
        self.declare_parameter('platform_current_status_timeout_sec', 0.8)
        self.declare_parameter('chassis_mode_timeout_sec', 1.0)
        self.declare_parameter('target_lost_timeout_sec', 1.0)
        self.declare_parameter('stage_timeout_sec', 25.0)

        self.declare_parameter('lift_direction', 1.0)
        self.declare_parameter('max_lift_speed_mmps', 25.0)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('lift_deadband_mm', 3.0)
        self.declare_parameter('z_tolerance_mm', 15.0)
        self.declare_parameter('stable_required_frames', 5)
        self.declare_parameter('platform_min_height_mm', 0.0)
        self.declare_parameter('platform_max_height_mm', 330.0)
        self.declare_parameter('actual_jump_resync_mm', 10.0)
        self.declare_parameter('auto_takeover_hold_ticks', 5)
        self.declare_parameter('debug_log_period_sec', 0.5)

        self.request_topic = str(self.get_parameter('request_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.startup_zero_done_topic = str(self.get_parameter('startup_zero_done_topic').value)
        self.datamatrix_topic = str(self.get_parameter('datamatrix_topic').value)
        self.platform_current_status_topic = str(self.get_parameter('platform_current_status_topic').value)
        self.platform_target_topic = str(self.get_parameter('platform_target_topic').value)
        self.chassis_control_mode_topic = str(self.get_parameter('chassis_control_mode_topic').value)
        self.visual_align_point_topic = str(self.get_parameter('visual_align_point_topic').value)
        self.align_u_px_current = float(self.get_parameter('align_u_px').value)
        self.align_v_px_current = float(self.get_parameter('align_v_px').value)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        done_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.startup_zero_done_pub = self.create_publisher(String, self.startup_zero_done_topic, done_qos)
        self.platform_target_pub = self.create_publisher(PlatformTarget, self.platform_target_topic, 10)

        self.req_sub = self.create_subscription(String, self.request_topic, self.on_request, 10)
        self.dm_sub = self.create_subscription(String, self.datamatrix_topic, self.on_datamatrix, 10)
        self.platform_sub = self.create_subscription(PlatformCurrentStatus, self.platform_current_status_topic, self.on_platform_current_status, 10)
        self.chassis_mode_sub = self.create_subscription(String, self.chassis_control_mode_topic, self.on_chassis_control_mode, 10)
        self.visual_align_sub = self.create_subscription(String, self.visual_align_point_topic, self.on_visual_align_point, 10)

        self.active = False
        self.state = 'IDLE'
        self.package_code = ''
        self.pickup_code = ''
        self.request_id = ''
        self.request_start_time = 0.0

        self.latest_dm_payload: Optional[Any] = None
        self.latest_dm_stamp = 0.0
        self.last_target_seen_time = 0.0

        self.platform_status_valid = False
        self.platform_status_stamp = 0.0
        self.actual_lift_z_mm = 0.0
        self.actual_slide_x_mm = 0.0
        self.actual_screw_x_mm = 0.0
        self.platform_control_mode = 0

        self.chassis_mode = 'unknown'
        self.chassis_mode_stamp = 0.0
        self.last_effective_mode = 'unknown'

        self.startup_zero_done = False
        self.startup_zero_start_time = 0.0
        self.startup_zero_last_log_time = 0.0

        self.previous_command_lift_z_mm: Optional[float] = None
        self.command_lift_z_mm: Optional[float] = None
        self.manual_synced_lift_z_mm: Optional[float] = None
        self.hold_ticks_remaining = 0

        # new_request_clear_previous_command_and_mode_edge
        # Clear stale command from previous run. The new request must start from actual height.
        self.previous_command_lift_z_mm = None
        self.command_lift_z_mm = None
        self.manual_synced_lift_z_mm = None
        self.last_effective_mode = 'unknown'
        self.startup_zero_done = False
        self.stable_count = 0
        self.last_target_center_px: Optional[Tuple[float, float]] = None
        self.last_log_time = 0.0

        self.startup_zero_command_mm: Optional[float] = None
        self.startup_zero_done_published = False
        self.startup_zero_stable_since = 0.0
        self.startup_zero_last_done_pub_time = 0.0
        self.startup_zero_last_mode = 'unknown'
        self.startup_zero_auto_hold_until = 0.0
        self.startup_zero_hold_height_mm = None

        hz = float(self.get_parameter('control_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, hz), self.on_timer)

        self.get_logger().info('oak_guided_pickup ready: hand-eye target + rm_serial_driver mode gating')
        self.publish_status('idle', 'idle', 'ready')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def pfloat(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def pint(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def pbool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def on_request(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            self.publish_status('failed', 'request', 'invalid_request_json')
            return

        command = str(data.get('command') or data.get('stage') or data.get('action') or 'lift_align').lower()
        if command in ('stop', 'cancel', 'halt'):
            self.active = False
            self.state = 'IDLE'
            self.publish_status('canceled', 'idle', 'request_cancel')
            return

        self.package_code = str(data.get('package_code') or data.get('pickup_code') or data.get('code') or '')
        self.pickup_code = str(data.get('pickup_code') or self.package_code)
        self.request_id = str(data.get('request_id') or f'lift_align_{int(time.time()*1000)}')
        self.request_start_time = self.now_sec()
        self.active = True
        self.state = 'SYNC_PLATFORM_ACTUAL_HEIGHT'
        self.stable_count = 0
        self.last_target_seen_time = self.now_sec()
        self.hold_ticks_remaining = 0
        if self.platform_status_valid:
            self.sync_to_actual_height()
        self.publish_status('running', 'sync_platform_actual_height', 'request_received')

    def on_datamatrix(self, msg: String) -> None:
        try:
            self.latest_dm_payload = json.loads(msg.data)
            self.latest_dm_stamp = self.now_sec()
        except Exception:
            return

    def on_platform_current_status(self, msg: PlatformCurrentStatus) -> None:
        self.platform_status_valid = True
        self.platform_status_stamp = self.now_sec()
        self.actual_lift_z_mm = float(msg.target_lift_z_mm)
        self.actual_slide_x_mm = float(msg.target_slide_x_mm)
        self.actual_screw_x_mm = float(msg.target_screw_x_mm)
        self.platform_control_mode = int(msg.control_mode)

        if self.active and self.previous_command_lift_z_mm is not None:
            gap = self.actual_lift_z_mm - self.previous_command_lift_z_mm
            if abs(gap) > self.pfloat('actual_jump_resync_mm'):
                self.sync_to_actual_height()
                self.hold_ticks_remaining = max(1, self.pint('auto_takeover_hold_ticks'))
                self.state = 'HOLD_CURRENT_HEIGHT_ON_TAKEOVER'
                self.publish_status('running', 'hold_current_height_on_takeover', 'actual_height_external_jump_resynced', {
                    'actual_command_gap_mm': gap,
                    'hold_ticks_remaining': self.hold_ticks_remaining,
                })

    def on_visual_align_point(self, msg: String) -> None:
        """Runtime slider update. Accepts JSON {align_u_px, align_v_px} or {u, v}."""
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            u = data.get('align_u_px', data.get('u', data.get('x')))
            v = data.get('align_v_px', data.get('v', data.get('y')))
            if u is not None:
                self.align_u_px_current = float(u)
            if v is not None:
                self.align_v_px_current = float(v)
            self.get_logger().info(
                f'[visual_align] align_u={self.align_u_px_current:.1f}, align_v={self.align_v_px_current:.1f}'
            )
        except Exception:
            pass

    def on_chassis_control_mode(self, msg: String) -> None:
        raw = str(msg.data).strip().lower()
        mode = 'unknown'
        try:
            data = json.loads(raw)
            if isinstance(data, dict):
                raw = str(data.get('mode') or data.get('mode_name') or data.get('state') or '').lower()
        except Exception:
            pass
        if 'manual' in raw or raw in ('1', 'm'):
            mode = 'manual'
        elif 'auto' in raw or raw in ('0', 'a'):
            mode = 'auto'
        self.chassis_mode = mode
        self.chassis_mode_stamp = self.now_sec()

    def effective_chassis_mode(self) -> str:
        now = self.now_sec()

        # Primary mode source: rm_serial_driver /chassis_control_mode.
        if self.chassis_mode != 'unknown' and now - self.chassis_mode_stamp <= self.pfloat('chassis_mode_timeout_sec'):
            return self.chassis_mode

        # Fallback mode source: rm_serial_driver /platform/current_status.control_mode.
        # This is still rm_serial data, not a new control-mode system.
        if self.pbool('use_platform_control_mode_fallback') and self.platform_status_valid:
            auto_val = self.pint('platform_auto_control_mode_value')
            manual_val = self.pint('platform_manual_control_mode_value')
            cmode = int(self.platform_control_mode)

            if cmode == manual_val:
                return 'manual'
            if cmode == auto_val:
                return 'auto'
            if self.pbool('platform_control_mode_as_auto_if_nonzero') and cmode != 0:
                return 'auto'

        if self.pbool('allow_control_when_chassis_mode_unknown'):
            return 'auto'
        return 'unknown'

    def sync_to_actual_height(self) -> bool:
        if not self.platform_status_valid:
            return False
        actual = clamp(self.actual_lift_z_mm, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm'))
        self.previous_command_lift_z_mm = actual
        self.command_lift_z_mm = actual
        self.manual_synced_lift_z_mm = actual
        return True

    def publish_platform_target(self, lift_z_mm: float, reason: str) -> None:
        msg = PlatformTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.target_lift_z_mm = float(clamp(lift_z_mm, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm')))
        msg.target_slide_x_mm = float(self.actual_slide_x_mm)
        msg.target_screw_x_mm = float(self.actual_screw_x_mm)
        self.platform_target_pub.publish(msg)

    def publish_hold_actual(self, reason: str) -> None:
        if self.platform_status_valid:
            self.publish_platform_target(self.actual_lift_z_mm, reason)

    def code_matches(self, code: str) -> bool:
        code = str(code)
        target = str(self.package_code)
        pickup = str(self.pickup_code)
        if target and code == target:
            return True
        if pickup and code == pickup:
            return True
        if target and code.endswith(target):
            return True
        if pickup and code.endswith(pickup):
            return True
        return False

    def get_target_observation(self) -> Optional[Tuple[float, float, Optional[Tuple[float, float, float]]]]:
        """Return target RGB center and optional camera xyz.

        Lift alignment uses only code_v_px - align_v_px. The xyz is kept for
        status/debug and does not drive platform height.
        """
        payload = self.latest_dm_payload
        if payload is None:
            return None
        for item in iter_detection_items(payload):
            code = str(item.get('code') or item.get('text') or item.get('data') or '')
            if not self.code_matches(code):
                continue
            center = parse_center_px(item)
            if center is None:
                continue
            xyz = parse_xyz_mm(item.get('camera_xyz_mm') or item.get('xyz_mm') or item.get('position_mm'))
            self.last_target_seen_time = self.now_sec()
            self.last_target_center_px = center
            return center[0], center[1], xyz
        return None

    def publish_startup_zero_done(self, reason: str, force: bool = False) -> None:
        """Publish startup-zero done.

        It is intentionally re-published as a heartbeat after done, because
        bringup gate helper processes may start after the first done message.
        """
        now = self.now_sec()
        hb = self.pfloat('startup_done_heartbeat_sec')
        if self.startup_zero_done_published and not force:
            if now - getattr(self, 'startup_zero_last_done_pub_time', 0.0) < hb:
                return

        msg = String()
        msg.data = json.dumps({
            'status': 'done',
            'stage': 'startup_feedback_zero',
            'reason': reason,
            'actual_lift_z_mm': float(self.actual_lift_z_mm),
            'target_lift_z_mm': float(self.pfloat('startup_safe_zero_target_mm')),
            'stamp': now,
        }, ensure_ascii=False)

        self.startup_zero_done_pub.publish(msg)
        self.startup_zero_done_published = True
        self.startup_zero_last_done_pub_time = now

    def handle_startup_safe_zero(self) -> None:
        """Startup feedback return-to-zero with manual-height synchronization.

        Required behavior:
        - Before /platform/current_status arrives: publish nothing.
        - In manual mode: publish target_lift_z_mm = actual_lift_z_mm.
          This keeps the PC platform command synchronized with the height moved by hand/RC.
        - When switching from manual to auto: hold the current feedback height for a short time.
        - Only after the auto hold period, descend toward 0 at max_lift_speed_mmps.
        - Startup gate opens only after feedback is near zero and stable.
        """
        now = self.now_sec()

        if self.startup_zero_done:
            self.publish_startup_zero_done('heartbeat_after_done', force=True)
            return

        if not self.pbool('enable_startup_safe_zero'):
            self.startup_zero_done = True
            self.publish_startup_zero_done('disabled', force=True)
            self.get_logger().info('[startup_feedback_zero] disabled, publish done gate')
            return

        if self.active:
            return

        if self.startup_zero_start_time <= 0.0:
            self.startup_zero_start_time = now

        # No feedback: do not publish anything.
        if (not self.platform_status_valid) or (now - self.platform_status_stamp > self.pfloat('platform_current_status_timeout_sec')):
            self.startup_zero_command_mm = None
            self.previous_command_lift_z_mm = None
            self.command_lift_z_mm = None
            self.startup_zero_stable_since = 0.0
            self.startup_zero_auto_hold_until = 0.0
            self.startup_zero_hold_height_mm = None

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().warn('[startup_feedback_zero] waiting for /platform/current_status feedback; no platform target published')
            return

        mode = self.effective_chassis_mode()
        actual = clamp(
            float(self.actual_lift_z_mm),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm')
        )
        target = clamp(
            self.pfloat('startup_safe_zero_target_mm'),
            self.pfloat('platform_min_height_mm'),
            self.pfloat('platform_max_height_mm'),
        )
        tol = self.pfloat('startup_safe_zero_tolerance_mm')
        confirm_sec = self.pfloat('startup_safe_zero_confirm_sec')

        # Manual mode: synchronize PC target to current hand-controlled height.
        # This is intentionally a publish, but it publishes the SAME height, not zero.
        if self.pbool('startup_safe_zero_require_auto') and mode != 'auto':
            command = actual

            self.startup_zero_command_mm = command
            self.previous_command_lift_z_mm = command
            self.command_lift_z_mm = command
            self.manual_synced_lift_z_mm = actual
            self.startup_zero_stable_since = 0.0
            self.startup_zero_auto_hold_until = 0.0
            self.startup_zero_hold_height_mm = None
            self.startup_zero_last_mode = mode
            self.startup_zero_start_time = now

            self.publish_platform_target(command, 'startup_manual_sync_same_height')

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().warn(
                    f'[startup_feedback_zero] manual sync, mode={mode}, '
                    f'actual={actual:.1f}, cmd={command:.1f}; publish same height, not zero'
                )
            return

        # Just switched into auto: hold current height first, do not descend immediately.
        if self.startup_zero_last_mode != 'auto':
            hold_sec = self.pfloat('startup_auto_hold_sec')
            self.startup_zero_auto_hold_until = now + hold_sec
            self.startup_zero_hold_height_mm = actual
            self.startup_zero_stable_since = 0.0
            self.startup_zero_start_time = now
            self.startup_zero_last_mode = 'auto'

            self.startup_zero_command_mm = actual
            self.previous_command_lift_z_mm = actual
            self.command_lift_z_mm = actual
            self.manual_synced_lift_z_mm = actual

            self.publish_platform_target(actual, 'startup_auto_takeover_hold_height')

            self.get_logger().info(
                f'[startup_feedback_zero] auto takeover, hold feedback height first: '
                f'actual={actual:.1f}, hold_sec={hold_sec:.1f}'
            )
            return

        self.startup_zero_last_mode = 'auto'

        # During auto takeover hold: keep target equal to current feedback height.
        if now < self.startup_zero_auto_hold_until:
            command = actual

            self.startup_zero_command_mm = command
            self.previous_command_lift_z_mm = command
            self.command_lift_z_mm = command
            self.manual_synced_lift_z_mm = actual
            self.startup_zero_stable_since = 0.0

            self.publish_platform_target(command, 'startup_auto_hold_same_height')

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                left = self.startup_zero_auto_hold_until - now
                self.get_logger().info(
                    f'[startup_feedback_zero] auto hold same height, '
                    f'actual={actual:.1f}, cmd={command:.1f}, hold_left={left:.1f}s'
                )
            return

        # Near zero: confirm stable zero. Do not keep forcing zero.
        if abs(actual - target) <= tol:
            self.startup_zero_command_mm = actual
            self.previous_command_lift_z_mm = actual
            self.command_lift_z_mm = actual
            self.manual_synced_lift_z_mm = actual

            if self.startup_zero_stable_since <= 0.0:
                self.startup_zero_stable_since = now

            stable_elapsed = now - self.startup_zero_stable_since

            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().info(
                    f'[startup_feedback_zero] zero confirming actual={actual:.1f} '
                    f'target={target:.1f} stable={stable_elapsed:.1f}/{confirm_sec:.1f}; no platform target published'
                )

            if stable_elapsed >= confirm_sec:
                self.startup_zero_done = True
                self.publish_startup_zero_done('reached_zero_by_feedback_stable', force=True)
                self.get_logger().info(
                    f'[startup_feedback_zero] done reason=reached_zero_by_feedback_stable actual={actual:.1f}'
                )
            return

        # Descend toward zero after auto hold.
        self.startup_zero_stable_since = 0.0
        max_step = self.pfloat('max_lift_speed_mmps') / max(1.0, self.pfloat('control_hz'))

        if target <= actual:
            command = max(target, actual - max_step)
        else:
            command = min(target, actual + max_step)

        command = clamp(command, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm'))

        self.startup_zero_command_mm = command
        self.previous_command_lift_z_mm = command
        self.command_lift_z_mm = command
        self.manual_synced_lift_z_mm = actual

        self.publish_platform_target(command, 'startup_feedback_zero_after_auto_hold')

        if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
            self.startup_zero_last_log_time = now
            self.get_logger().info(
                f'[startup_feedback_zero] descending after hold, mode={mode}, '
                f'actual={actual:.1f}, cmd={command:.1f}, target={target:.1f}, max_step={max_step:.2f}'
            )

        if now - self.startup_zero_start_time > self.pfloat('startup_safe_zero_timeout_sec'):
            if now - self.startup_zero_last_log_time > self.pfloat('debug_log_period_sec'):
                self.startup_zero_last_log_time = now
                self.get_logger().error(
                    '[startup_feedback_zero] timeout; NOT opening startup gate because lift is not confirmed zero'
                )
            return

    def on_timer(self) -> None:
        if not self.active:
            self.handle_startup_safe_zero()
            return

        now = self.now_sec()
        if now - self.request_start_time > self.pfloat('stage_timeout_sec'):
            self.active = False
            self.publish_status('failed', self.state, 'stage_timeout')
            return

        if not self.platform_status_valid or now - self.platform_status_stamp > self.pfloat('platform_current_status_timeout_sec'):
            self.publish_status('running', 'sync_platform_actual_height', 'waiting_platform_current_status')
            return

        mode = self.effective_chassis_mode()
        if self.pbool('require_chassis_auto_for_platform_control') and mode != 'auto':
            self.sync_to_actual_height()
            self.last_effective_mode = mode
            self.publish_status('running', 'manual_sync', 'rm_serial_driver_not_auto_sync_actual_only', {
                'effective_chassis_mode': mode,
            })
            return

        if self.last_effective_mode != 'auto':
            self.sync_to_actual_height()
            self.hold_ticks_remaining = max(1, self.pint('auto_takeover_hold_ticks'))
            self.state = 'HOLD_CURRENT_HEIGHT_ON_TAKEOVER'
            self.last_effective_mode = 'auto'
            self.publish_status('running', 'hold_current_height_on_takeover', 'manual_to_auto_sync_actual_height', {
                'hold_ticks_remaining': self.hold_ticks_remaining,
            })
            return

        if self.state == 'SYNC_PLATFORM_ACTUAL_HEIGHT':
            self.sync_to_actual_height()
            self.hold_ticks_remaining = max(1, self.pint('auto_takeover_hold_ticks'))
            self.state = 'HOLD_CURRENT_HEIGHT_ON_TAKEOVER'
            self.publish_status('running', 'hold_current_height_on_takeover', 'actual_height_synced_before_auto', {
                'hold_ticks_remaining': self.hold_ticks_remaining,
            })
            return

        if self.state == 'HOLD_CURRENT_HEIGHT_ON_TAKEOVER':
            self.publish_hold_actual('auto_takeover_hold_actual')
            self.hold_ticks_remaining -= 1
            if self.hold_ticks_remaining > 0:
                self.publish_status('running', 'hold_current_height_on_takeover', 'holding_actual_height_before_visual_target', {
                    'hold_ticks_remaining': self.hold_ticks_remaining,
                })
                return
            self.state = 'LIFT_ALIGN_WITH_HAND_EYE_TARGET'

        obs = self.get_target_observation()
        if obs is None:
            lost = now - self.last_target_seen_time
            if lost > self.pfloat('target_lost_timeout_sec'):
                self.active = False
                self.publish_status('failed', 'lift_align', 'target_lost_timeout_or_no_center_px', {'target_lost_sec': lost})
            else:
                self.publish_hold_actual('waiting_target_center_px_hold_actual')
                self.publish_status('running', 'lift_align', 'waiting_target_center_px', {'target_lost_sec': lost})
            return

        code_u_px, code_v_px, xyz = obs
        error_v_px = float(code_v_px) - float(self.align_v_px_current)
        desired = self.actual_lift_z_mm + self.pfloat('lift_pixel_direction') * self.pfloat('kv_to_lift_mm_per_px') * error_v_px
        if self.previous_command_lift_z_mm is None:
            self.sync_to_actual_height()
        max_step = self.pfloat('max_lift_speed_mmps') / max(1.0, self.pfloat('control_hz'))
        if abs(error_v_px) < self.pfloat('v_tolerance_px'):
            command = self.actual_lift_z_mm
        else:
            delta = desired - float(self.previous_command_lift_z_mm)
            command = float(self.previous_command_lift_z_mm) + clamp(delta, -max_step, max_step)
        command = clamp(command, self.pfloat('platform_min_height_mm'), self.pfloat('platform_max_height_mm'))
        self.command_lift_z_mm = command
        self.previous_command_lift_z_mm = command
        self.publish_platform_target(command, 'rgb_slider_v_lift_align_limited_command')

        if abs(error_v_px) <= self.pfloat('v_tolerance_px'):
            self.stable_count += 1
        else:
            self.stable_count = 0

        extra = {
            'effective_chassis_mode': mode,
            'code_u_px': code_u_px,
            'code_v_px': code_v_px,
            'align_u_px': self.align_u_px_current,
            'align_v_px': self.align_v_px_current,
            'error_v_px': error_v_px,
            'camera_xyz_mm': {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]} if xyz is not None else None,
            'desired_lift_z_mm': desired,
            'command_lift_z_mm': command,
            'actual_lift_z_mm': self.actual_lift_z_mm,
            'max_step_mm': max_step,
        }
        if self.stable_count >= self.pint('stable_required_frames'):
            self.active = False
            self.publish_hold_actual('lift_align_succeeded_hold_actual')
            self.publish_status('succeeded', 'lift_align', 'lift_aligned_to_rgb_slider_point', extra)
            return
        self.publish_status('running', 'lift_align', 'tracking_rgb_slider_align_v', extra)

        if now - self.last_log_time > self.pfloat('debug_log_period_sec'):
            self.last_log_time = now
            self.get_logger().info(
                f"[rgb_slider_lift] mode={mode} actual={self.actual_lift_z_mm:.1f} code_v={code_v_px:.1f} align_v={self.align_v_px_current:.1f} error_v={error_v_px:.1f}px cmd={command:.1f}"
            )

    def publish_status(self, status: str, stage: str, reason: str, extra: Optional[Dict[str, Any]] = None) -> None:
        data: Dict[str, Any] = {
            'stamp': self.now_sec(),
            'status': status,
            'stage': stage,
            'reason': reason,
            'state': self.state,
            'request_id': self.request_id,
            'package_code': self.package_code,
            'pickup_code': self.pickup_code,
            'actual_lift_z_mm': self.actual_lift_z_mm if self.platform_status_valid else None,
            'command_lift_z_mm': self.command_lift_z_mm,
            'previous_command_lift_z_mm': self.previous_command_lift_z_mm,
            'manual_synced_lift_z_mm': self.manual_synced_lift_z_mm,
            'chassis_mode': self.chassis_mode,
            'effective_chassis_mode': self.effective_chassis_mode() if self.active else self.chassis_mode,
            'platform_status_valid': self.platform_status_valid,
            'stable_count': self.stable_count,
        }
        if extra:
            data.update(extra)
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OakGuidedPickupNode()
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
