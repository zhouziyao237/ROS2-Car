from dataclasses import dataclass, field
from typing import Dict, List
import time
import re

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from std_msgs.msg import String, Int32, Bool

from yolo_msg.msg import YoloDetectionArray, YoloDetection, Box2D
from ocr_msg.msg import OcrRequestArray, OcrRequest, OcrResultArray

from target_matcher_msg.msg import (
    OcrLine,
    OcrCache as OcrCacheMsg,
    TrackedTarget,
    TrackedTargetArray,
    MatchedTarget,
)

from .kalman_tracker import KalmanTracker, Track, bbox_to_list
from .matcher_utils import (
    merge_ocr_lines,
    match_target_code,
    normalize_code,
    score_ocr_text,
)


@dataclass
class OcrCache:
    ocr_done: bool = False
    ocr_pending: bool = False
    lines: List[str] = field(default_factory=list)
    confidences: List[float] = field(default_factory=list)
    indices_list: List[List[int]] = field(default_factory=list)
    merged_text: str = ""
    norm_text: str = ""
    avg_conf: float = 0.0
    score: float = 0.0
    last_ocr_time: float = 0.0
    last_request_time: float = 0.0
    fail_count: int = 0
    matched_token: str = ""


class TargetMatcherNode(Node):
    def __init__(self):
        super().__init__("target_matcher_node")

        # =====================
        # Basic topic parameters
        # =====================
        self.declare_parameter("yolo_topic", "/yolov5/detections")
        self.declare_parameter("ocr_request_topic", "/vision/ocr_requests")
        self.declare_parameter("ocr_result_topic", "/vision/ocr_results")
        self.declare_parameter("target_code_topic", "/target_code")
        self.declare_parameter("enable_topic", "/target_matcher/enable")

        self.declare_parameter("matched_index_topic", "/target_matcher/matched_index")
        self.declare_parameter("matched_detection_topic", "/target_matcher/matched_detection")
        self.declare_parameter("tracked_detections_topic", "/target_matcher/tracked_detections")
        self.declare_parameter("debug_text_topic", "/target_matcher/debug_text")

        self.declare_parameter("tracked_targets_topic", "/target_matcher/tracked_targets")
        self.declare_parameter("matched_target_topic", "/target_matcher/matched_target")

        # =====================
        # Tracking parameters
        # =====================
        self.declare_parameter("track_iou_threshold", 0.12)
        self.declare_parameter("track_center_distance_threshold", 1.0)
        self.declare_parameter("max_missing_frames", 25)
        self.declare_parameter("min_yolo_confidence", 0.25)

        self.declare_parameter("only_need_ocr", False)

        # =====================
        # OCR policy
        # =====================
        self.declare_parameter("ocr_min_confidence", 0.35)
        self.declare_parameter("ocr_retry_interval_sec", 8.0)
        self.declare_parameter("ocr_cache_timeout_sec", 60.0)
        self.declare_parameter("ocr_pending_timeout_sec", 15.0)
        self.declare_parameter("avoid_repeat_ocr", True)

        # 这个参数保留，但高清映射后建议 1.15~1.25
        self.declare_parameter("ocr_roi_expand_scale", 1.20)

        # 兼容旧参数：现在 image_width/image_height 代表 OCR 图像尺寸
        self.declare_parameter("image_width", 1920)
        self.declare_parameter("image_height", 1080)

        # =====================
        # YOLO bbox -> OCR high-res image mapping
        # =====================
        # YOLO bbox 坐标系，例如 OAK NN preview 640x640
        self.declare_parameter("yolo_image_width", 640)
        self.declare_parameter("yolo_image_height", 640)

        # OCR 实际切图的高清图尺寸，例如 OAK video 1920x1080
        self.declare_parameter("ocr_image_width", 1920)
        self.declare_parameter("ocr_image_height", 1080)

        # True = 将 YOLO bbox 从 640x640 映射到 1920x1080 再发 OCR
        self.declare_parameter("map_yolo_bbox_to_ocr_image", True)

        # 非等比例扩框：横向多扩，上下少扩，适合 4 位横排数字
        self.declare_parameter("ocr_roi_expand_x", 1.45)
        self.declare_parameter("ocr_roi_expand_y", 1.15)

        # OCR throttling
        self.declare_parameter("max_ocr_requests_per_frame", 1)
        self.declare_parameter("min_track_age_for_ocr", 8)

        # =====================
        # Runtime control
        # =====================
        self.declare_parameter("reuse_track_indices", True)
        self.declare_parameter("tracking_enabled", False)
        self.declare_parameter("auto_enable_on_target_code", True)
        self.declare_parameter("require_target_code_for_ocr", True)

        self.declare_parameter("publish_debug_text", True)
        self.declare_parameter("target_code", "")

        # =====================
        # Recognition mode / state machine
        # =====================
        self.declare_parameter("recognition_mode", "lock_then_pose")
        self.declare_parameter("recognition_mode_topic", "/target_matcher/recognition_mode")

        self.declare_parameter("state_topic", "/target_matcher/state")
        self.declare_parameter("arrived_topic", "/target_matcher/arrived")
        self.declare_parameter("reset_lock_topic", "/target_matcher/reset_lock")
        self.declare_parameter("calibration_enable_topic", "/target_matcher/calibration_enable")

        self.declare_parameter("stable_match_required_count", 2)
        self.declare_parameter("lock_lost_timeout_sec", 5.0)
        self.declare_parameter("hold_calibration_after_arrived", True)

        # =====================
        # Load parameters
        # =====================
        self.yolo_topic = str(self.get_parameter("yolo_topic").value)
        self.ocr_request_topic = str(self.get_parameter("ocr_request_topic").value)
        self.ocr_result_topic = str(self.get_parameter("ocr_result_topic").value)
        self.target_code_topic = str(self.get_parameter("target_code_topic").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)

        self.matched_index_topic = str(self.get_parameter("matched_index_topic").value)
        self.matched_detection_topic = str(self.get_parameter("matched_detection_topic").value)
        self.tracked_detections_topic = str(self.get_parameter("tracked_detections_topic").value)
        self.debug_text_topic = str(self.get_parameter("debug_text_topic").value)
        self.tracked_targets_topic = str(self.get_parameter("tracked_targets_topic").value)
        self.matched_target_topic = str(self.get_parameter("matched_target_topic").value)

        self.track_iou_threshold = float(self.get_parameter("track_iou_threshold").value)
        self.track_center_distance_threshold = float(
            self.get_parameter("track_center_distance_threshold").value
        )
        self.max_missing_frames = int(self.get_parameter("max_missing_frames").value)
        self.min_yolo_confidence = float(self.get_parameter("min_yolo_confidence").value)
        self.only_need_ocr = bool(self.get_parameter("only_need_ocr").value)

        self.ocr_min_confidence = float(self.get_parameter("ocr_min_confidence").value)
        self.ocr_retry_interval_sec = float(self.get_parameter("ocr_retry_interval_sec").value)
        self.ocr_cache_timeout_sec = float(self.get_parameter("ocr_cache_timeout_sec").value)
        self.ocr_pending_timeout_sec = float(self.get_parameter("ocr_pending_timeout_sec").value)
        self.avoid_repeat_ocr = bool(self.get_parameter("avoid_repeat_ocr").value)

        self.ocr_roi_expand_scale = float(self.get_parameter("ocr_roi_expand_scale").value)

        self.yolo_image_width = int(self.get_parameter("yolo_image_width").value)
        self.yolo_image_height = int(self.get_parameter("yolo_image_height").value)
        self.ocr_image_width = int(self.get_parameter("ocr_image_width").value)
        self.ocr_image_height = int(self.get_parameter("ocr_image_height").value)
        self.map_yolo_bbox_to_ocr_image = bool(
            self.get_parameter("map_yolo_bbox_to_ocr_image").value
        )

        self.ocr_roi_expand_x = float(self.get_parameter("ocr_roi_expand_x").value)
        self.ocr_roi_expand_y = float(self.get_parameter("ocr_roi_expand_y").value)

        # OCR ROI 边界使用高清图尺寸
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)

        if self.map_yolo_bbox_to_ocr_image:
            self.image_width = self.ocr_image_width
            self.image_height = self.ocr_image_height

        self.max_ocr_requests_per_frame = int(
            self.get_parameter("max_ocr_requests_per_frame").value
        )
        self.min_track_age_for_ocr = int(
            self.get_parameter("min_track_age_for_ocr").value
        )

        self.reuse_track_indices = bool(self.get_parameter("reuse_track_indices").value)
        self.tracking_enabled = bool(self.get_parameter("tracking_enabled").value)
        self.auto_enable_on_target_code = bool(self.get_parameter("auto_enable_on_target_code").value)
        self.require_target_code_for_ocr = bool(
            self.get_parameter("require_target_code_for_ocr").value
        )

        self.publish_debug_text = bool(self.get_parameter("publish_debug_text").value)
        self.target_code = str(self.get_parameter("target_code").value or "").strip()

        self.recognition_mode = str(
            self.get_parameter("recognition_mode").value
        ).strip()

        self.recognition_mode_topic = str(
            self.get_parameter("recognition_mode_topic").value
        )

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.arrived_topic = str(self.get_parameter("arrived_topic").value)
        self.reset_lock_topic = str(self.get_parameter("reset_lock_topic").value)
        self.calibration_enable_topic = str(
            self.get_parameter("calibration_enable_topic").value
        )

        self.stable_match_required_count = int(
            self.get_parameter("stable_match_required_count").value
        )
        self.lock_lost_timeout_sec = float(
            self.get_parameter("lock_lost_timeout_sec").value
        )
        self.hold_calibration_after_arrived = bool(
            self.get_parameter("hold_calibration_after_arrived").value
        )

        if self.recognition_mode not in ["continuous", "lock_then_pose", "pose_only"]:
            self.get_logger().warn(
                f"Invalid recognition_mode={self.recognition_mode}, fallback to lock_then_pose."
            )
            self.recognition_mode = "lock_then_pose"

        if self.auto_enable_on_target_code and self.target_code:
            self.tracking_enabled = True

        # =====================
        # Tracker
        # =====================
        self.tracker = KalmanTracker(
            iou_threshold=self.track_iou_threshold,
            max_missing_frames=self.max_missing_frames,
            center_distance_threshold=self.track_center_distance_threshold,
        )

        self.ret_data = [[], [], []]
        self.track_id_to_index: Dict[str, int] = {}
        self.free_indices: List[int] = []
        self.last_header = None

        # =====================
        # State machine runtime
        # =====================
        self.state = "IDLE"

        self.robot_arrived = False
        self.calibration_enabled = False

        self.locked_track_id = ""
        self.locked_index = -1
        self.locked_token = ""
        self.locked_text = ""
        self.lock_start_time = 0.0
        self.last_lock_seen_time = 0.0

        self.stable_candidate_index = -1
        self.stable_candidate_token = ""
        self.stable_candidate_count = 0
        self.stable_candidate_last_ocr_time = 0.0

        # =====================
        # Subscribers
        # =====================
        self.create_subscription(
            YoloDetectionArray,
            self.yolo_topic,
            self.on_yolo_detections,
            10,
        )

        self.create_subscription(
            OcrResultArray,
            self.ocr_result_topic,
            self.on_ocr_results,
            10,
        )

        self.create_subscription(
            String,
            self.target_code_topic,
            self.on_target_code,
            10,
        )

        self.create_subscription(
            Bool,
            self.enable_topic,
            self.on_enable,
            10,
        )

        self.create_subscription(
            String,
            self.recognition_mode_topic,
            self.on_recognition_mode,
            10,
        )

        self.create_subscription(
            Bool,
            self.arrived_topic,
            self.on_arrived,
            10,
        )

        self.create_subscription(
            Bool,
            self.reset_lock_topic,
            self.on_reset_lock,
            10,
        )

        # =====================
        # Publishers
        # =====================
        self.ocr_request_pub = self.create_publisher(
            OcrRequestArray,
            self.ocr_request_topic,
            10,
        )

        self.matched_index_pub = self.create_publisher(
            Int32,
            self.matched_index_topic,
            10,
        )

        self.matched_detection_pub = self.create_publisher(
            YoloDetectionArray,
            self.matched_detection_topic,
            10,
        )

        self.tracked_detections_pub = self.create_publisher(
            YoloDetectionArray,
            self.tracked_detections_topic,
            10,
        )

        self.debug_text_pub = self.create_publisher(
            String,
            self.debug_text_topic,
            10,
        )

        self.tracked_targets_pub = self.create_publisher(
            TrackedTargetArray,
            self.tracked_targets_topic,
            10,
        )

        self.matched_target_pub = self.create_publisher(
            MatchedTarget,
            self.matched_target_topic,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10,
        )

        self.calibration_enable_pub = self.create_publisher(
            Bool,
            self.calibration_enable_topic,
            10,
        )

        self.get_logger().info("target_matcher_node started.")
        self.get_logger().info(f"yolo_topic={self.yolo_topic}")
        self.get_logger().info(f"ocr_request_topic={self.ocr_request_topic}")
        self.get_logger().info(f"ocr_result_topic={self.ocr_result_topic}")
        self.get_logger().info(f"target_code_topic={self.target_code_topic}")
        self.get_logger().info(f"enable_topic={self.enable_topic}")
        self.get_logger().info(f"tracked_targets_topic={self.tracked_targets_topic}")
        self.get_logger().info(f"matched_target_topic={self.matched_target_topic}")

        self.get_logger().info(f"tracking_enabled={self.tracking_enabled}")
        self.get_logger().info(f"only_need_ocr={self.only_need_ocr}")
        self.get_logger().info(f"min_yolo_confidence={self.min_yolo_confidence}")
        self.get_logger().info(f"ocr_roi_expand_scale={self.ocr_roi_expand_scale}")
        self.get_logger().info(f"ocr_roi_expand_x={self.ocr_roi_expand_x}")
        self.get_logger().info(f"ocr_roi_expand_y={self.ocr_roi_expand_y}")
        self.get_logger().info(f"require_target_code_for_ocr={self.require_target_code_for_ocr}")
        self.get_logger().info(f"avoid_repeat_ocr={self.avoid_repeat_ocr}")
        self.get_logger().info(f"ocr_retry_interval_sec={self.ocr_retry_interval_sec}")
        self.get_logger().info(f"ocr_pending_timeout_sec={self.ocr_pending_timeout_sec}")
        self.get_logger().info(f"max_ocr_requests_per_frame={self.max_ocr_requests_per_frame}")
        self.get_logger().info(f"min_track_age_for_ocr={self.min_track_age_for_ocr}")

        self.get_logger().info(f"map_yolo_bbox_to_ocr_image={self.map_yolo_bbox_to_ocr_image}")
        self.get_logger().info(f"yolo_image_size={self.yolo_image_width}x{self.yolo_image_height}")
        self.get_logger().info(f"ocr_image_size={self.ocr_image_width}x{self.ocr_image_height}")
        self.get_logger().info(f"roi_boundary_size={self.image_width}x{self.image_height}")

        self.get_logger().info(f"recognition_mode={self.recognition_mode}")
        self.get_logger().info(f"recognition_mode_topic={self.recognition_mode_topic}")
        self.get_logger().info(f"state_topic={self.state_topic}")
        self.get_logger().info(f"arrived_topic={self.arrived_topic}")
        self.get_logger().info(f"reset_lock_topic={self.reset_lock_topic}")
        self.get_logger().info(f"calibration_enable_topic={self.calibration_enable_topic}")
        self.get_logger().info(f"stable_match_required_count={self.stable_match_required_count}")
        self.get_logger().info(f"lock_lost_timeout_sec={self.lock_lost_timeout_sec}")
        self.get_logger().info(f"hold_calibration_after_arrived={self.hold_calibration_after_arrived}")

        if self.target_code:
            self.set_state("SEARCHING")
        else:
            self.set_state("IDLE")

    # =====================
    # State machine helpers
    # =====================
    def set_state(self, state: str):
        state = str(state)

        if self.state == state:
            return

        self.state = state

        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

        self.get_logger().info(f"state updated: {self.state}")

    def publish_calibration_enable(self, enabled: bool):
        enabled = bool(enabled)

        if self.calibration_enabled == enabled:
            return

        self.calibration_enabled = enabled

        msg = Bool()
        msg.data = self.calibration_enabled
        self.calibration_enable_pub.publish(msg)

        self.get_logger().info(f"calibration_enable={self.calibration_enabled}")

    def on_recognition_mode(self, msg: String):
        mode = str(msg.data or "").strip()

        if mode not in ["continuous", "lock_then_pose", "pose_only"]:
            self.get_logger().warn(
                f"Invalid recognition_mode: {mode}. "
                f"Allowed: continuous, lock_then_pose, pose_only"
            )
            return

        self.recognition_mode = mode

        self.get_logger().info(
            f"recognition_mode updated: {self.recognition_mode}"
        )

        if self.recognition_mode == "pose_only":
            self.publish_debug("Recognition mode changed to pose_only: OCR disabled.")

    def on_arrived(self, msg: Bool):
        self.robot_arrived = bool(msg.data)

        if self.robot_arrived:
            self.get_logger().info("Robot arrived signal received.")

            if self.locked_track_id:
                self.set_state("CALIBRATION_ENABLED")
                self.publish_calibration_enable(True)
            else:
                self.set_state("ARRIVED_NO_LOCK")
                self.publish_calibration_enable(False)
        else:
            self.get_logger().info("Robot arrived signal cleared.")
            self.publish_calibration_enable(False)

            if self.locked_track_id:
                self.set_state("APPROACHING")
            elif self.target_code:
                self.set_state("SEARCHING")
            else:
                self.set_state("IDLE")

        self.publish_outputs()

    def on_reset_lock(self, msg: Bool):
        if bool(msg.data):
            self.reset_search_session("reset topic received")

    def reset_lock(self, reason: str = ""):
        old_track_id = self.locked_track_id

        self.locked_track_id = ""
        self.locked_index = -1
        self.locked_token = ""
        self.locked_text = ""
        self.lock_start_time = 0.0
        self.last_lock_seen_time = 0.0

        self.stable_candidate_index = -1
        self.stable_candidate_token = ""
        self.stable_candidate_count = 0
        self.stable_candidate_last_ocr_time = 0.0

        self.ret_data[2] = []

        self.robot_arrived = False
        self.publish_calibration_enable(False)

        if self.target_code.strip():
            self.set_state("SEARCHING")
        else:
            self.set_state("IDLE")

        self.get_logger().info(
            f"Target lock reset. old_track_id={old_track_id}, reason={reason}"
        )

        self.publish_outputs()

    def reset_search_session(self, reason: str = ""):
        old_track_id = self.locked_track_id

        self.locked_track_id = ""
        self.locked_index = -1
        self.locked_token = ""
        self.locked_text = ""
        self.lock_start_time = 0.0
        self.last_lock_seen_time = 0.0

        self.stable_candidate_index = -1
        self.stable_candidate_token = ""
        self.stable_candidate_count = 0
        self.stable_candidate_last_ocr_time = 0.0

        self.robot_arrived = False
        self.publish_calibration_enable(False)

        self.tracker = KalmanTracker(
            iou_threshold=self.track_iou_threshold,
            max_missing_frames=self.max_missing_frames,
            center_distance_threshold=self.track_center_distance_threshold,
        )

        self.ret_data = [[], [], []]
        self.track_id_to_index.clear()
        self.free_indices.clear()

        if self.target_code.strip() and self.tracking_enabled:
            self.set_state("SEARCHING")
        elif self.target_code.strip():
            self.set_state("SEARCHING")
        else:
            self.set_state("IDLE")

        self.get_logger().info(
            f"Full search session reset. old_track_id={old_track_id}, reason={reason}"
        )

        self.publish_outputs()

    def lock_target(self, index: int, token: str, text: str):
        if index < 0 or index >= len(self.ret_data[0]):
            return

        track_info = self.ret_data[0][index]

        if not track_info:
            return

        self.locked_track_id = str(track_info["track_id"])
        self.locked_index = int(index)
        self.locked_token = str(token)
        self.locked_text = str(text)
        self.lock_start_time = time.time()
        self.last_lock_seen_time = self.lock_start_time

        self.ret_data[2] = [index]

        if index < len(self.ret_data[1]) and isinstance(self.ret_data[1][index], OcrCache):
            self.ret_data[1][index].matched_token = str(token)

        self.set_state("APPROACHING")

        self.get_logger().info(
            f"TARGET LOCKED: track_id={self.locked_track_id}, "
            f"index={index}, token={token}, text={text}"
        )

        self.publish_outputs()

    # =====================
    # Callbacks
    # =====================
    def on_enable(self, msg: Bool):
        old_enabled = self.tracking_enabled
        self.tracking_enabled = bool(msg.data)

        if not self.tracking_enabled:
            self.reset_search_session("tracking disabled")
            self.publish_debug("Tracking task disabled. Search session cleared.")
        else:
            if not old_enabled:
                self.reset_search_session("tracking enabled")

            if self.target_code:
                self.set_state("SEARCHING")

            self.publish_debug("Tracking task enabled.")

        self.get_logger().info(f"tracking_enabled updated: {self.tracking_enabled}")
        self.publish_outputs()

    def on_target_code(self, msg: String):
        new_code = str(msg.data or "").strip()
        old_code = str(self.target_code or "").strip()

        self.target_code = new_code

        if self.auto_enable_on_target_code and self.target_code:
            self.tracking_enabled = True

        if new_code != old_code:
            self.reset_search_session("target_code changed")
        else:
            if self.target_code and self.tracking_enabled:
                self.set_state("SEARCHING")
            elif not self.target_code:
                self.reset_search_session("target_code cleared")

        self.get_logger().info(
            f"target_code updated: {self.target_code}, tracking_enabled={self.tracking_enabled}"
        )

        self.update_matches()
        self.publish_outputs()

    def on_yolo_detections(self, msg: YoloDetectionArray):
        self.last_header = msg.header

        detections = self.convert_yolo_detections(msg)

        if len(detections) == 0:
            self.tracker.update([])
            self.remove_dead_tracks(self.tracker.removed_track_ids)
            self.update_matches()
            self.publish_outputs()
            return

        tracks = self.tracker.update(detections)
        self.remove_dead_tracks(self.tracker.removed_track_ids)

        if self.locked_track_id:
            locked_tracks = [
                t for t in tracks
                if str(t.track_id) == str(self.locked_track_id)
            ]

            if locked_tracks:
                tracks_sorted = locked_tracks
            else:
                tracks_sorted = sorted(
                    tracks,
                    key=lambda t: float(t.confidence),
                    reverse=True,
                )
        else:
            tracks_sorted = sorted(
                tracks,
                key=lambda t: float(t.confidence),
                reverse=True,
            )

        ocr_sent = 0

        for track in tracks_sorted:
            self.ensure_track_slot(track)
            self.update_tracking_slot(track)

            if ocr_sent >= self.max_ocr_requests_per_frame:
                continue

            if self.should_request_ocr(track):
                self.publish_ocr_request(track, msg.header)
                ocr_sent += 1

        self.update_matches()
        self.publish_outputs()

    def on_ocr_results(self, msg: OcrResultArray):
        grouped = {}

        for result in msg.results:
            target_id = str(result.target_id)
            grouped.setdefault(target_id, []).append(result)

        for track_id, results in grouped.items():
            if track_id not in self.track_id_to_index:
                self.get_logger().warn(
                    f"Drop stale OCR result: track_id={track_id} no longer exists"
                )
                continue

            index = self.track_id_to_index[track_id]
            self.ensure_cache_slot(index)

            lines = []
            confidences = []
            indices_list = []

            for result in results:
                text = str(result.raw_text or "").strip()

                if not bool(result.has_text):
                    continue

                if not text:
                    continue

                lines.append(text)
                confidences.append(float(result.confidence))
                indices_list.append([int(x) for x in result.indices])

            cache: OcrCache = self.ret_data[1][index]

            if not lines:
                cache.ocr_pending = False
                cache.fail_count += 1
                self.get_logger().warn(
                    f"OCR returned no valid text: track_id={track_id}, index={index}, "
                    f"fail_count={cache.fail_count}"
                )
                continue

            merged_text = merge_ocr_lines(lines)
            avg_conf = sum(confidences) / max(1, len(confidences))
            new_score = score_ocr_text(lines, confidences)

            if cache.ocr_done and new_score < cache.score:
                cache.ocr_pending = False
                continue

            cache.ocr_done = True
            cache.ocr_pending = False
            cache.lines = lines
            cache.confidences = confidences
            cache.indices_list = indices_list
            cache.merged_text = merged_text
            cache.norm_text = normalize_code(merged_text)
            cache.avg_conf = float(avg_conf)
            cache.score = float(new_score)
            cache.last_ocr_time = time.time()
            cache.fail_count = 0
            cache.matched_token = ""

            self.get_logger().info(
                f"OCR cache updated: track_id={track_id}, index={index}, "
                f"text={merged_text}, avg_conf={avg_conf:.3f}"
            )

        self.update_matches()
        self.publish_outputs()

    # =====================
    # YOLO conversion
    # =====================
    def convert_yolo_detections(self, msg: YoloDetectionArray) -> List[dict]:
        converted = []

        for det in msg.detections:
            confidence = float(det.confidence)

            if confidence < self.min_yolo_confidence:
                continue

            bbox = bbox_to_list(det.box)

            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue

            converted.append({
                "bbox": bbox,
                "class_name": str(det.class_name),
                "class_id": int(det.class_id),
                "confidence": confidence,
                "need_ocr": True,
                "source_target_id": str(det.target_id),
            })

        return converted

    # =====================
    # BBox mapping
    # =====================
    def map_bbox_to_ocr_image(self, bbox: List[float]) -> List[float]:
        if not self.map_yolo_bbox_to_ocr_image:
            return list(bbox)

        sx = float(self.ocr_image_width) / max(1.0, float(self.yolo_image_width))
        sy = float(self.ocr_image_height) / max(1.0, float(self.yolo_image_height))

        x1, y1, x2, y2 = bbox

        return [
            x1 * sx,
            y1 * sy,
            x2 * sx,
            y2 * sy,
        ]

    def map_bbox_to_yolo_image(self, bbox: List[float]) -> List[float]:
        if not self.map_yolo_bbox_to_ocr_image:
            return list(bbox)

        sx = float(self.yolo_image_width) / max(1.0, float(self.ocr_image_width))
        sy = float(self.yolo_image_height) / max(1.0, float(self.ocr_image_height))

        x1, y1, x2, y2 = bbox

        return [
            x1 * sx,
            y1 * sy,
            x2 * sx,
            y2 * sy,
        ]

    # =====================
    # Track slot management
    # =====================
    def allocate_index(self, fallback_index: int) -> int:
        if self.reuse_track_indices and self.free_indices:
            return int(self.free_indices.pop(0))

        return int(fallback_index)

    def ensure_track_slot(self, track: Track):
        if track.track_id in self.track_id_to_index:
            return

        index = self.allocate_index(track.index)
        self.track_id_to_index[track.track_id] = index

        while len(self.ret_data[0]) <= index:
            self.ret_data[0].append(None)

        while len(self.ret_data[1]) <= index:
            self.ret_data[1].append(OcrCache())

        self.ret_data[0][index] = None
        self.ret_data[1][index] = OcrCache()

        self.get_logger().info(
            f"New track slot: track_id={track.track_id}, index={index}"
        )

    def ensure_cache_slot(self, index: int):
        while len(self.ret_data[1]) <= index:
            self.ret_data[1].append(OcrCache())

        if self.ret_data[1][index] is None:
            self.ret_data[1][index] = OcrCache()

    def update_tracking_slot(self, track: Track):
        index = self.track_id_to_index[track.track_id]

        raw_bbox = list(track.bbox)
        mapped_bbox = self.map_bbox_to_ocr_image(raw_bbox)

        self.ret_data[0][index] = {
            "track_id": track.track_id,
            "index": index,
            "bbox": mapped_bbox,
            "raw_yolo_bbox": raw_bbox,
            "class_name": track.class_name,
            "class_id": int(track.class_id),
            "confidence": float(track.confidence),
            "missing_frames": int(track.missing_frames),
            "age": int(track.age),
            "need_ocr": True,
        }

    def remove_dead_tracks(self, removed_track_ids: List[str]):
        for track_id in removed_track_ids:
            if track_id not in self.track_id_to_index:
                continue

            index = self.track_id_to_index.pop(track_id)

            if index < len(self.ret_data[0]):
                self.ret_data[0][index] = None

            if index < len(self.ret_data[1]):
                self.ret_data[1][index] = OcrCache()

            self.ret_data[2] = [idx for idx in self.ret_data[2] if idx != index]

            if self.locked_track_id == track_id:
                self.last_lock_seen_time = self.last_lock_seen_time or time.time()

            if self.reuse_track_indices and index not in self.free_indices:
                self.free_indices.append(index)
                self.free_indices.sort()

            self.get_logger().info(
                f"Removed track: track_id={track_id}, index={index}; "
                f"free_indices={len(self.free_indices)}"
            )

    # =====================
    # OCR request logic
    # =====================
    def should_request_ocr(self, track: Track) -> bool:
        if not self.tracking_enabled:
            return False

        if self.recognition_mode == "pose_only":
            return False

        if self.recognition_mode == "lock_then_pose" and self.locked_track_id:
            return False

        if int(track.age) < self.min_track_age_for_ocr:
            return False

        if self.require_target_code_for_ocr and not self.target_code.strip():
            return False

        index = self.track_id_to_index[track.track_id]
        cache: OcrCache = self.ret_data[1][index]
        now = time.time()

        if cache.ocr_pending:
            if now - cache.last_request_time < self.ocr_pending_timeout_sec:
                return False

            cache.ocr_pending = False
            cache.fail_count += 1
            self.get_logger().warn(
                f"OCR pending timeout: track_id={track.track_id}, "
                f"index={index}, fail_count={cache.fail_count}"
            )

        if cache.ocr_done and cache.avg_conf >= self.ocr_min_confidence:
            if self.avoid_repeat_ocr:
                return False

            if self.ocr_cache_timeout_sec <= 0:
                return False

            if now - cache.last_ocr_time < self.ocr_cache_timeout_sec:
                return False

        if now - cache.last_request_time < self.ocr_retry_interval_sec:
            return False

        return True

    def expand_bbox(self, bbox: List[float], scale: float) -> List[float]:
        x1, y1, x2, y2 = bbox

        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        width = max(1.0, x2 - x1)
        height = max(1.0, y2 - y1)

        # 优先使用非等比例扩框
        # 横排数字：左右多扩，上下少扩
        scale_x = max(float(scale), float(self.ocr_roi_expand_x))
        scale_y = float(self.ocr_roi_expand_y)

        new_width = width * scale_x
        new_height = height * scale_y

        nx1 = max(0.0, cx - new_width * 0.5)
        ny1 = max(0.0, cy - new_height * 0.5)
        nx2 = min(float(self.image_width - 1), cx + new_width * 0.5)
        ny2 = min(float(self.image_height - 1), cy + new_height * 0.5)

        return [nx1, ny1, nx2, ny2]

    def publish_ocr_request(self, track: Track, header):
        index = self.track_id_to_index[track.track_id]
        cache: OcrCache = self.ret_data[1][index]

        raw_bbox_640 = list(track.bbox)
        mapped_bbox = self.map_bbox_to_ocr_image(raw_bbox_640)
        ocr_bbox = self.expand_bbox(mapped_bbox, self.ocr_roi_expand_scale)

        req_array = OcrRequestArray()
        req_array.header = header

        req = OcrRequest()
        req.header = header
        req.target_id = str(track.track_id)
        req.class_name = str(track.class_name)
        req.yolo_confidence = float(track.confidence)
        req.need_ocr = True
        req.box = self.make_box2d_from_bbox(ocr_bbox)

        req_array.requests.append(req)

        cache.ocr_pending = True
        cache.last_request_time = time.time()

        self.ocr_request_pub.publish(req_array)

        text = (
            f"OCR request sent: track_id={track.track_id}, index={index}, "
            f"confidence={track.confidence:.3f}, "
            f"raw_bbox_640={raw_bbox_640}, mapped_bbox={mapped_bbox}, ocr_bbox={ocr_bbox}"
        )

        self.get_logger().info(text)
        self.publish_debug(text)

    # =====================
    # Strict numeric matching helpers
    # =====================
    def is_numeric_target_code(self, value: str) -> bool:
        value = str(value or "").strip()
        return bool(value) and re.fullmatch(r"\d+", value) is not None

    def strict_numeric_match(self, target_code: str, ocr_text: str):
        target = re.sub(r"\D", "", str(target_code or ""))
        if not target:
            return False, ""

        tokens = re.findall(r"\d+", str(ocr_text or ""))
        for token in tokens:
            if token == target:
                return True, token

        return False, ""

    # =====================
    # Matching with lock state machine
    # =====================
    def update_matches(self):
        now = time.time()

        self.ret_data[2] = []

        if not self.tracking_enabled:
            self.set_state("IDLE")
            return

        if not self.target_code.strip():
            self.set_state("IDLE")
            return

        if self.locked_track_id:
            if self.locked_track_id in self.track_id_to_index:
                index = self.track_id_to_index[self.locked_track_id]

                if index < len(self.ret_data[0]) and self.ret_data[0][index]:
                    self.locked_index = int(index)
                    self.last_lock_seen_time = now
                    self.ret_data[2] = [index]

                    if self.robot_arrived:
                        self.set_state("CALIBRATION_ENABLED")
                        self.publish_calibration_enable(True)
                    else:
                        self.set_state("APPROACHING")

                    return

            lost_dt = now - self.last_lock_seen_time

            if lost_dt < self.lock_lost_timeout_sec:
                self.set_state("LOCK_LOST_HOLD")
                self.ret_data[2] = []
                return

            if self.robot_arrived and self.hold_calibration_after_arrived:
                self.set_state("CALIBRATION_ENABLED")
                self.publish_calibration_enable(True)
                return

            self.reset_search_session(
                f"locked target lost timeout: {lost_dt:.2f}s"
            )
            return

        best_index = -1
        best_token = ""
        best_text = ""
        best_score = -1.0
        best_ocr_time = 0.0

        for index, cache in enumerate(self.ret_data[1]):
            if cache is None:
                continue

            if not isinstance(cache, OcrCache):
                continue

            if not cache.ocr_done:
                continue

            if index >= len(self.ret_data[0]) or not self.ret_data[0][index]:
                continue

            if self.is_numeric_target_code(self.target_code):
                ok, token = self.strict_numeric_match(self.target_code, cache.merged_text)
            else:
                ok, token = match_target_code(self.target_code, cache.merged_text)

            if not ok:
                cache.matched_token = ""
                continue

            cache.matched_token = str(token)

            score = float(cache.score) + float(cache.avg_conf)

            if score > best_score:
                best_score = score
                best_index = index
                best_token = token
                best_text = cache.merged_text
                best_ocr_time = cache.last_ocr_time

        if best_index < 0:
            self.set_state("SEARCHING")
            self.stable_candidate_index = -1
            self.stable_candidate_token = ""
            self.stable_candidate_count = 0
            self.stable_candidate_last_ocr_time = 0.0

            for cache in self.ret_data[1]:
                if isinstance(cache, OcrCache):
                    cache.matched_token = ""

            return

        if (
            best_index == self.stable_candidate_index
            and best_token == self.stable_candidate_token
        ):
            if best_ocr_time > self.stable_candidate_last_ocr_time:
                self.stable_candidate_count += 1
                self.stable_candidate_last_ocr_time = best_ocr_time
        else:
            self.stable_candidate_index = best_index
            self.stable_candidate_token = best_token
            self.stable_candidate_count = 1
            self.stable_candidate_last_ocr_time = best_ocr_time

        self.set_state("CANDIDATE_FOUND")

        self.get_logger().info(
            f"candidate match: index={best_index}, token={best_token}, "
            f"count={self.stable_candidate_count}/{self.stable_match_required_count}, "
            f"text={best_text}"
        )

        if self.stable_candidate_count >= self.stable_match_required_count:
            self.lock_target(best_index, best_token, best_text)

    # =====================
    # Publish outputs
    # =====================
    def publish_outputs(self):
        self.publish_matched_index()
        self.publish_all_tracked_detections()
        self.publish_all_tracked_targets()
        self.publish_matched_detection_array()
        self.publish_matched_target()

    def publish_matched_index(self):
        msg = Int32()
        msg.data = int(self.ret_data[2][0]) if self.ret_data[2] else -1
        self.matched_index_pub.publish(msg)

    def publish_all_tracked_detections(self):
        arr = YoloDetectionArray()

        if self.last_header is not None:
            arr.header = self.last_header

        for track_info in self.ret_data[0]:
            if track_info:
                arr.detections.append(self.make_detection_from_track_info(track_info))

        self.tracked_detections_pub.publish(arr)

    def publish_all_tracked_targets(self):
        arr = TrackedTargetArray()

        if self.last_header is not None:
            arr.header = self.last_header

        for index, track_info in enumerate(self.ret_data[0]):
            if not track_info:
                continue

            cache = self.ret_data[1][index] if index < len(self.ret_data[1]) else OcrCache()
            arr.targets.append(self.make_tracked_target_msg(index, track_info, cache))

        self.tracked_targets_pub.publish(arr)

    def publish_matched_detection_array(self):
        arr = YoloDetectionArray()

        if self.last_header is not None:
            arr.header = self.last_header

        if self.ret_data[2]:
            index = int(self.ret_data[2][0])

            if index < len(self.ret_data[0]) and self.ret_data[0][index]:
                arr.detections.append(
                    self.make_detection_from_track_info(self.ret_data[0][index])
                )

        self.matched_detection_pub.publish(arr)

    def publish_matched_target(self):
        msg = MatchedTarget()

        if self.last_header is not None:
            msg.header = self.last_header

        msg.target_code = str(self.target_code)
        msg.matched = False
        msg.matched_index = -1
        msg.box = self.make_box2d_from_bbox([0.0, 0.0, 0.0, 0.0])
        msg.ocr_cache = self.make_ocr_cache_msg(OcrCache())

        if self.ret_data[2]:
            index = int(self.ret_data[2][0])

            if index < len(self.ret_data[0]) and self.ret_data[0][index]:
                track_info = self.ret_data[0][index]
                cache = self.ret_data[1][index]

                msg.matched = True
                msg.matched_index = index
                msg.track_id = str(track_info["track_id"])
                msg.target_code = str(self.target_code)

                if isinstance(cache, OcrCache):
                    msg.matched_token = str(cache.matched_token)
                    msg.ocr_cache = self.make_ocr_cache_msg(cache)
                else:
                    msg.matched_token = ""
                    msg.ocr_cache = self.make_ocr_cache_msg(OcrCache())

                msg.class_name = str(track_info["class_name"])
                msg.class_id = int(track_info["class_id"])
                msg.yolo_confidence = float(track_info["confidence"])
                msg.need_ocr = bool(track_info["need_ocr"])
                msg.box = self.make_box2d_from_bbox(track_info["bbox"])
                msg.age = int(track_info.get("age", 0))
                msg.missing_frames = int(track_info.get("missing_frames", 0))

        self.matched_target_pub.publish(msg)

    # =====================
    # Message helpers
    # =====================
    def make_tracked_target_msg(
        self,
        index: int,
        track_info: dict,
        cache: OcrCache,
    ) -> TrackedTarget:
        msg = TrackedTarget()

        if self.last_header is not None:
            msg.header = self.last_header

        msg.index = int(index)
        msg.track_id = str(track_info["track_id"])
        msg.class_name = str(track_info["class_name"])
        msg.class_id = int(track_info["class_id"])
        msg.yolo_confidence = float(track_info["confidence"])
        msg.need_ocr = bool(track_info["need_ocr"])
        msg.box = self.make_box2d_from_bbox(track_info["bbox"])
        msg.age = int(track_info.get("age", 0))
        msg.missing_frames = int(track_info.get("missing_frames", 0))

        msg.has_ocr = bool(isinstance(cache, OcrCache) and cache.ocr_done)
        msg.ocr_cache = self.make_ocr_cache_msg(
            cache if isinstance(cache, OcrCache) else OcrCache()
        )

        msg.matched = int(index) in self.ret_data[2]
        msg.matched_token = str(cache.matched_token if isinstance(cache, OcrCache) else "")

        return msg

    def make_ocr_cache_msg(self, cache: OcrCache) -> OcrCacheMsg:
        msg = OcrCacheMsg()

        msg.ocr_done = bool(cache.ocr_done)
        msg.ocr_pending = bool(cache.ocr_pending)
        msg.merged_text = str(cache.merged_text)
        msg.norm_text = str(cache.norm_text)
        msg.avg_confidence = float(cache.avg_conf)
        msg.score = float(cache.score)
        msg.fail_count = int(cache.fail_count)
        msg.last_ocr_stamp = self.float_time_to_ros_time(cache.last_ocr_time)

        for line_id, text in enumerate(cache.lines):
            line_msg = OcrLine()
            line_msg.line_id = int(line_id)
            line_msg.text = str(text)
            line_msg.confidence = (
                float(cache.confidences[line_id])
                if line_id < len(cache.confidences)
                else 0.0
            )

            if line_id < len(cache.indices_list):
                line_msg.indices = [int(x) for x in cache.indices_list[line_id]]

            msg.lines.append(line_msg)

        return msg

    def float_time_to_ros_time(self, value: float) -> RosTime:
        msg = RosTime()

        if not value or value <= 0:
            return msg

        sec = int(value)
        msg.sec = sec
        msg.nanosec = int((float(value) - sec) * 1e9)

        return msg

    def make_detection_from_track_info(self, track_info) -> YoloDetection:
        det = YoloDetection()

        if self.last_header is not None:
            det.header = self.last_header

        det.target_id = str(track_info["track_id"])
        det.class_name = str(track_info["class_name"])
        det.class_id = int(track_info["class_id"])
        det.confidence = float(track_info["confidence"])
        det.box = self.make_box2d_from_bbox(track_info["bbox"])
        det.need_ocr = bool(track_info.get("need_ocr", True))

        return det

    def make_box2d_from_bbox(self, bbox: List[float]) -> Box2D:
        x1, y1, x2, y2 = bbox

        box = Box2D()
        box.x1 = int(round(x1))
        box.y1 = int(round(y1))
        box.x2 = int(round(x2))
        box.y2 = int(round(y2))
        box.center_u = int(round((x1 + x2) * 0.5))
        box.center_v = int(round((y1 + y2) * 0.5))
        box.width = int(round(max(0.0, x2 - x1)))
        box.height = int(round(max(0.0, y2 - y1)))

        return box

    def publish_debug(self, text: str):
        if not self.publish_debug_text:
            return

        msg = String()
        msg.data = str(text)
        self.debug_text_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TargetMatcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()