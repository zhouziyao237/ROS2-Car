import os
import queue
import threading
import time
from collections import OrderedDict

import cv2
import numpy as np
import onnxruntime as ort
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ocr_msg.msg import OcrRequest, OcrRequestArray, OcrResult, OcrResultArray
from yolo_msg.msg import Box2D


class OcrNode(Node):
    def __init__(self):
        super().__init__('ocr_node')

        self.declare_parameter('input_mode', 'topic')
        self.declare_parameter('image_path', '')
        self.declare_parameter('save_path', '')
        self.declare_parameter('exit_after_image', True)

        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('request_topic', '/vision/ocr_requests')
        self.declare_parameter('result_topic', '/vision/ocr_results')
        self.declare_parameter('debug_image_topic', '/vision/ocr_debug_image')

        self.declare_parameter('det_model', 'weights/det_static_qdq_int8.onnx')
        self.declare_parameter('cls_model', 'weights/cls_static_qdq_int8.onnx')
        self.declare_parameter('rec_model', 'weights/rec_fp32.onnx')
        self.declare_parameter('rec_yml', 'weights/inference.yml')
        self.declare_parameter('use_cls', False)

        self.declare_parameter('det_limit_side_len', 960)
        self.declare_parameter('det_db_thresh', 0.3)
        self.declare_parameter('det_db_box_thresh', 0.6)
        self.declare_parameter('det_db_unclip_ratio', 1.5)
        self.declare_parameter('det_min_size', 3)
        self.declare_parameter('drop_score', 0.5)

        self.declare_parameter('rec_batch_size', 6)
        self.declare_parameter('rec_img_h', 48)
        self.declare_parameter('rec_max_img_w', 320)
        self.declare_parameter('rec_use_dynamic_width', False)

        self.declare_parameter('crop_padding_ratio', 0.08)
        self.declare_parameter('cls_thresh', 0.9)

        self.declare_parameter('ort_num_threads', 1)
        self.declare_parameter('det_num_threads', 1)
        self.declare_parameter('cls_num_threads', 1)
        self.declare_parameter('rec_num_threads', 4)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('save_debug_image', False)
        self.declare_parameter('log_recognized_text', False)

        self.declare_parameter('min_roi_width', 30)
        self.declare_parameter('min_roi_height', 15)

        self.declare_parameter('image_cache_size', 10)
        self.declare_parameter('max_image_time_diff_ms', 80.0)
        self.declare_parameter('use_nearest_image_if_exact_stamp_missing', True)
        self.declare_parameter('ocr_queue_size', 8)

        self.input_mode = self.get_parameter('input_mode').value
        self.image_path = self.get_parameter('image_path').value
        self.save_path = self.get_parameter('save_path').value
        self.exit_after_image = bool(self.get_parameter('exit_after_image').value)

        self.image_topic = self.get_parameter('image_topic').value
        self.request_topic = self.get_parameter('request_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.det_model = self.resolve_path(self.get_parameter('det_model').value)
        self.cls_model = self.resolve_path(self.get_parameter('cls_model').value)
        self.rec_model = self.resolve_path(self.get_parameter('rec_model').value)
        self.rec_yml = self.resolve_path(self.get_parameter('rec_yml').value)
        self.use_cls = bool(self.get_parameter('use_cls').value)

        self.det_limit_side_len = int(self.get_parameter('det_limit_side_len').value)
        self.det_db_thresh = float(self.get_parameter('det_db_thresh').value)
        self.det_db_box_thresh = float(self.get_parameter('det_db_box_thresh').value)
        self.det_db_unclip_ratio = float(self.get_parameter('det_db_unclip_ratio').value)
        self.det_min_size = int(self.get_parameter('det_min_size').value)
        self.drop_score = float(self.get_parameter('drop_score').value)

        self.rec_batch_size = int(self.get_parameter('rec_batch_size').value)
        self.rec_img_h = int(self.get_parameter('rec_img_h').value)
        self.rec_max_img_w = int(self.get_parameter('rec_max_img_w').value)
        self.rec_use_dynamic_width = bool(self.get_parameter('rec_use_dynamic_width').value)

        self.crop_padding_ratio = float(self.get_parameter('crop_padding_ratio').value)
        self.cls_thresh = float(self.get_parameter('cls_thresh').value)

        self.ort_num_threads = int(self.get_parameter('ort_num_threads').value)
        self.det_num_threads = int(self.get_parameter('det_num_threads').value)
        self.cls_num_threads = int(self.get_parameter('cls_num_threads').value)
        self.rec_num_threads = int(self.get_parameter('rec_num_threads').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.save_debug_image = bool(self.get_parameter('save_debug_image').value)
        self.log_recognized_text = bool(self.get_parameter('log_recognized_text').value)

        self.min_roi_width = int(self.get_parameter('min_roi_width').value)
        self.min_roi_height = int(self.get_parameter('min_roi_height').value)

        self.image_cache_size = int(self.get_parameter('image_cache_size').value)
        self.max_image_time_diff_ns = int(float(self.get_parameter('max_image_time_diff_ms').value) * 1e6)
        self.use_nearest = bool(self.get_parameter('use_nearest_image_if_exact_stamp_missing').value)
        self.ocr_queue_size = max(1, int(self.get_parameter('ocr_queue_size').value))

        self.bridge = CvBridge()
        self.image_cache = OrderedDict()
        self.image_cache_lock = threading.Lock()
        self.request_queue = None
        self.request_worker = None
        self.request_worker_stop = threading.Event()
        self.has_run_single_image = False
        self.warned_unclip_dependency = False

        self.get_logger().info('Loading ONNX models...')
        self.det_sess = self.make_session(self.det_model, self.det_num_threads)
        if self.use_cls:
            self.cls_sess = self.make_session(self.cls_model, self.cls_num_threads)
            self.cls_input_name = self.cls_sess.get_inputs()[0].name
            self.cls_output_name = self.cls_sess.get_outputs()[0].name
        else:
            self.cls_sess = None
            self.cls_input_name = None
            self.cls_output_name = None
        self.rec_sess = self.make_session(self.rec_model, self.rec_num_threads)

        self.det_input_name = self.det_sess.get_inputs()[0].name
        self.det_output_name = self.det_sess.get_outputs()[0].name
        self.rec_input_name = self.rec_sess.get_inputs()[0].name
        self.rec_output_name = self.rec_sess.get_outputs()[0].name

        self.index_to_char = self.load_index_from_rec_yml(self.rec_yml)
        self.rec_output_dim = self.rec_sess.get_outputs()[0].shape[-1]
        if isinstance(self.rec_output_dim, int) and self.rec_output_dim != len(self.index_to_char):
            self.get_logger().warn('rec output dim does not match YAML character_dict + blank')

        self.result_pub = self.create_publisher(OcrResultArray, self.result_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.real_mode = self.input_mode
        if self.input_mode == 'auto':
            if self.image_path and os.path.exists(self.image_path):
                self.real_mode = 'image'
            else:
                self.real_mode = 'topic'

        if self.real_mode not in ['topic', 'image']:
            raise RuntimeError('input_mode must be topic, image, or auto.')

        self.get_logger().info('OCR node started.')
        self.get_logger().info(f'requested input_mode={self.input_mode}')
        self.get_logger().info(f'selected input_mode={self.real_mode}')
        self.get_logger().info(f'result_topic={self.result_topic}')
        self.get_logger().info(f'use_cls={self.use_cls}')
        self.get_logger().info(f'rec_batch_size={self.rec_batch_size}')
        self.get_logger().info(
            'onnxruntime threads: '
            f'det={self.det_num_threads}, cls={self.cls_num_threads}, rec={self.rec_num_threads}'
        )
        self.get_logger().info(f'publish_debug_image={self.publish_debug_image}')
        self.get_logger().info(f'save_debug_image={self.save_debug_image}')
        self.get_logger().info(f'log_recognized_text={self.log_recognized_text}')

        if self.real_mode == 'topic':
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.request_sub = self.create_subscription(
                OcrRequestArray,
                self.request_topic,
                self.request_callback,
                10,
            )
            self.request_queue = queue.Queue(maxsize=self.ocr_queue_size)
            self.request_worker = threading.Thread(target=self.request_worker_loop, daemon=True)
            self.request_worker.start()

            self.get_logger().info('OCR node started with timestamp image cache.')
            self.get_logger().info(f'image_topic={self.image_topic}')
            self.get_logger().info(f'request_topic={self.request_topic}')
            self.get_logger().info(
                f'image_cache_size={self.image_cache_size}, max_diff_ns={self.max_image_time_diff_ns}'
            )
            self.get_logger().info(f'ocr_queue_size={self.ocr_queue_size}')
        else:
            self.get_logger().info('Single image OCR mode.')
            self.get_logger().info(f'image_path={self.image_path}')
            self.get_logger().info(f'save_path={self.save_path}')
            self.timer = self.create_timer(0.2, self.single_image_callback)

    def destroy_node(self):
        self.request_worker_stop.set()
        if self.request_worker is not None and self.request_worker.is_alive():
            self.request_worker.join(timeout=0.5)
        return super().destroy_node()

    def request_shutdown(self):
        threading.Thread(target=self._deferred_shutdown, daemon=True).start()

    @staticmethod
    def _deferred_shutdown():
        time.sleep(0.05)
        if rclpy.ok():
            rclpy.shutdown()

    def build_full_image_request(self, image_bgr, frame_id, stamp):
        image_h, image_w = image_bgr.shape[:2]

        req = OcrRequest()
        req.header.stamp = stamp
        req.header.frame_id = frame_id
        req.target_id = 'single_image_0'
        req.class_name = 'full_image'
        req.yolo_confidence = 1.0
        req.need_ocr = True
        req.box = Box2D(
            x1=0,
            y1=0,
            x2=image_w,
            y2=image_h,
            center_u=image_w // 2,
            center_v=image_h // 2,
            width=image_w,
            height=image_h,
        )
        return req

    def resolve_path(self, path):
        if os.path.isabs(path):
            return path

        pkg_share = get_package_share_directory('ocr_node')
        return os.path.join(pkg_share, path)

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    # def make_session(self, model_path, num_threads=1):
    #     if not os.path.exists(model_path):
    #         raise FileNotFoundError(f'Model not found: {model_path}')

    #     so = ort.SessionOptions()
    #     so.intra_op_num_threads = int(num_threads)
    #     so.inter_op_num_threads = 1
    #     so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    #     return ort.InferenceSession(model_path, sess_options=so, providers=['CPUExecutionProvider'])

    def make_session(self, model_path, num_threads=1):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'Model not found: {model_path}')

        so = ort.SessionOptions()
        so.intra_op_num_threads = int(num_threads)
        so.inter_op_num_threads = 1

        # 关键修复：
        # 当前 OCR det/cls 只有 INT8/QDQ ONNX 模型。
        # ORT_ENABLE_ALL 可能把 Conv 优化成 NCHWc 路径，
        # 然后触发:
        #   p2o.pd_op.relu.*_nchwc
        #   GetElementType is not implemented
        # 所以这里禁用图优化，优先保证稳定运行。
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL

        self.get_logger().info(
            f'Creating ONNXRuntime session: model={model_path}, '
            f'threads={num_threads}, graph_optimization=ORT_DISABLE_ALL'
        )

        return ort.InferenceSession(
            model_path,
            sess_options=so,
            providers=['CPUExecutionProvider'],
        )

    def load_index_from_rec_yml(self, yml_path):
        if not os.path.exists(yml_path):
            raise FileNotFoundError(f'rec inference.yml not found: {yml_path}')

        with open(yml_path, 'r', encoding='utf-8') as file_obj:
            data = yaml.safe_load(file_obj)

        post_process = data.get('PostProcess', {})
        character_dict = post_process.get('character_dict', None)
        if character_dict is None:
            raise RuntimeError('No PostProcess.character_dict found in inference.yml')

        if isinstance(character_dict, str):
            chars = list(character_dict)
        elif isinstance(character_dict, (list, tuple)):
            chars = list(character_dict)
        else:
            raise RuntimeError('PostProcess.character_dict must be a list, tuple, or string')

        chars = [str(ch) for ch in chars]
        use_space_char = bool(post_process.get('use_space_char', False))
        if use_space_char:
            chars.append(' ')

        index_to_char = [''] + chars
        rec_dim = self.rec_sess.get_outputs()[0].shape[-1]

        self.get_logger().info(f'character_dict len from yaml: {len(character_dict)}')
        self.get_logger().info(f'use_space_char: {use_space_char}')
        self.get_logger().info(f'num_classes with blank: {len(index_to_char)}')
        self.get_logger().info(f'rec output dim: {rec_dim}, yaml classes: {len(index_to_char)}')
        if isinstance(rec_dim, int) and rec_dim != len(index_to_char):
            self.get_logger().warn(
                'rec output dim does not match PostProcess.character_dict + blank. '
                f'rec output dim={rec_dim}, yaml classes={len(index_to_char)}'
            )

        return index_to_char

    def image_callback(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge image conversion failed: {exc}')
            return

        stamp_ns = self.stamp_to_ns(msg.header.stamp)
        with self.image_cache_lock:
            self.image_cache[stamp_ns] = (msg.header, image_bgr)
            while len(self.image_cache) > self.image_cache_size:
                self.image_cache.popitem(last=False)

    def get_image_by_stamp(self, stamp):
        target_stamp_ns = self.stamp_to_ns(stamp)

        with self.image_cache_lock:
            if not self.image_cache:
                return None, None, None

            if target_stamp_ns in self.image_cache:
                header, image_bgr = self.image_cache[target_stamp_ns]
                return header, image_bgr.copy(), 0

            if not self.use_nearest:
                return None, None, None

            nearest_stamp_ns = min(self.image_cache.keys(), key=lambda value: abs(value - target_stamp_ns))
            diff_ns = abs(nearest_stamp_ns - target_stamp_ns)
            if diff_ns > self.max_image_time_diff_ns:
                return None, None, diff_ns

            header, image_bgr = self.image_cache[nearest_stamp_ns]
            return header, image_bgr.copy(), diff_ns

    def enqueue_request_task(self, task):
        if self.request_queue is None:
            return

        if self.request_queue.full():
            try:
                dropped_task = self.request_queue.get_nowait()
                self.request_queue.task_done()
                self.get_logger().warn(
                    'OCR task queue full, dropping oldest request batch '
                    f'with {len(dropped_task["requests"])} requests.'
                )
            except queue.Empty:
                pass

        try:
            self.request_queue.put_nowait(task)
        except queue.Full:
            self.get_logger().warn('OCR task queue still full after drop, skipping newest request batch.')

    def request_worker_loop(self):
        while not self.request_worker_stop.is_set():
            try:
                task = self.request_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self.process_request_task(task)
            except Exception as exc:
                self.get_logger().error(f'OCR request worker failed: {exc}')
            finally:
                self.request_queue.task_done()

    def make_box2d_from_points(self, points, offset_x=0, offset_y=0):
        pts = np.array(points, dtype=np.float32)

        x1 = int(np.min(pts[:, 0]) + offset_x)
        y1 = int(np.min(pts[:, 1]) + offset_y)
        x2 = int(np.max(pts[:, 0]) + offset_x)
        y2 = int(np.max(pts[:, 1]) + offset_y)

        box = Box2D()
        box.x1 = x1
        box.y1 = y1
        box.x2 = x2
        box.y2 = y2
        box.center_u = int((x1 + x2) / 2)
        box.center_v = int((y1 + y2) / 2)
        box.width = int(x2 - x1)
        box.height = int(y2 - y1)
        return box

    def sort_text_boxes(self, boxes, y_thresh_ratio=0.5):
        if not boxes:
            return []

        items = []
        for box in boxes:
            box = np.array(box, dtype=np.float32)
            x_min = float(np.min(box[:, 0]))
            y_min = float(np.min(box[:, 1]))
            y_max = float(np.max(box[:, 1]))
            cy = (y_min + y_max) / 2.0
            h = y_max - y_min
            items.append({
                'box': box,
                'x_min': x_min,
                'cy': cy,
                'h': h,
            })

        items.sort(key=lambda item: item['cy'])

        lines = []
        for item in items:
            placed = False
            for line in lines:
                line_cy = np.mean([line_item['cy'] for line_item in line])
                line_h = np.mean([line_item['h'] for line_item in line])
                if abs(item['cy'] - line_cy) < max(10.0, line_h * y_thresh_ratio):
                    line.append(item)
                    placed = True
                    break
            if not placed:
                lines.append([item])

        sorted_boxes = []
        for line in lines:
            line.sort(key=lambda item: item['x_min'])
            sorted_boxes.extend([line_item['box'] for line_item in line])
        return sorted_boxes

    def request_callback(self, msg):
        _, image_bgr, diff_ns = self.get_image_by_stamp(msg.header.stamp)

        if image_bgr is None:
            self.get_logger().warn('No synchronized image found for OCR request.')
            return

        if diff_ns and diff_ns > 0:
            self.get_logger().debug(f'Using nearest RGB image for OCR request. diff_ms={diff_ns / 1e6:.2f}')

        if not msg.requests:
            return

        self.enqueue_request_task({
            'header': msg.header,
            'image_bgr': image_bgr,
            'requests': list(msg.requests),
        })

    def process_request_task(self, task):
        msg_header = task['header']
        image_bgr = task['image_bgr']
        requests = task['requests']

        result_array = OcrResultArray()
        result_array.header = msg_header

        need_debug_image = self.publish_debug_image or (self.save_debug_image and bool(self.save_path))
        debug_image = image_bgr.copy() if need_debug_image else None

        for req in requests:
            if not req.need_ocr:
                continue

            if self.stamp_to_ns(req.header.stamp) == 0 and not req.header.frame_id:
                req.header = msg_header

            x1 = max(0, int(req.box.x1))
            y1 = max(0, int(req.box.y1))
            x2 = min(image_bgr.shape[1], int(req.box.x2))
            y2 = min(image_bgr.shape[0], int(req.box.y2))

            if x2 <= x1 or y2 <= y1:
                self.get_logger().warn(f'Invalid OCR ROI for {req.target_id}: [{x1}, {y1}, {x2}, {y2}]')
                continue

            if (x2 - x1) < self.min_roi_width or (y2 - y1) < self.min_roi_height:
                self.get_logger().debug(f'Skipping small OCR ROI for {req.target_id}: [{x1}, {y1}, {x2}, {y2}]')
                continue

            roi = image_bgr[y1:y2, x1:x2]
            line_results = self.run_ocr_lines_on_roi(
                roi=roi,
                req=req,
                offset_x=x1,
                offset_y=y1,
            )

            result_array.results.extend(line_results)

            if debug_image is not None:
                self.draw_roi_box(debug_image, req.box, req.target_id)
                self.draw_ocr_results(debug_image, line_results)

        self.result_pub.publish(result_array)

        if debug_image is not None and self.publish_debug_image:
            self.publish_debug_image_msg(debug_image, msg_header)

        if debug_image is not None and self.save_debug_image:
            self.save_debug_image_file(debug_image, announce=False)

    def run_ocr_lines_on_roi(self, roi, req, offset_x=0, offset_y=0):
        results = []

        det_x, scale_w, scale_h = self.preprocess_det(roi)
        det_out = self.det_sess.run(
            [self.det_output_name],
            {self.det_input_name: det_x},
        )[0]

        if det_out.ndim == 4 and det_out.shape[1] == 1:
            prob_map = det_out[0, 0]
        elif det_out.ndim == 4 and det_out.shape[-1] == 1:
            prob_map = det_out[0, :, :, 0]
        else:
            raise RuntimeError(f'Unexpected det output shape: {det_out.shape}')

        boxes, _ = self.det_postprocess(prob_map, scale_w, scale_h)

        if self.log_recognized_text:
            self.get_logger().info(f'target_id={req.target_id}, OCR text boxes={len(boxes)}')

        prepared = []
        for text_box_points in boxes:
            crop = self.crop_rotate_image(roi, text_box_points)
            if crop is None:
                continue

            crop = self.add_white_padding(crop)
            if crop.size == 0:
                continue

            if self.use_cls:
                crop, cls_id, cls_score = self.run_cls(crop)
            else:
                cls_id = 0
                cls_score = 1.0

            prepared.append({
                'crop': crop,
                'points': text_box_points,
                'cls_id': cls_id,
                'cls_score': cls_score,
            })

        if not prepared:
            return results

        crops = [item['crop'] for item in prepared]
        texts, indices_list, confs = self.run_rec_batch(crops)
        if len(texts) != len(prepared):
            self.get_logger().warn(
                f'rec batch output size mismatch. prepared={len(prepared)}, decoded={len(texts)}'
            )

        line_id = 0
        for item, text, indices, confidence in zip(prepared, texts, indices_list, confs):

            if not text.strip():
                continue

            if confidence < self.drop_score:
                continue

            result = OcrResult()
            result.header = req.header
            result.target_id = req.target_id
            result.line_id = int(line_id)
            result.raw_text = text
            result.confidence = float(confidence)
            result.indices = [int(index) for index in indices]
            result.has_text = True
            result.roi_box = req.box
            result.text_box = self.make_box2d_from_points(
                item['points'],
                offset_x=offset_x,
                offset_y=offset_y,
            )
            results.append(result)

            if self.log_recognized_text:
                self.get_logger().info(
                    f'[{req.target_id} line {line_id}] text={text}, score={confidence:.4f}, '
                    f"cls_id={item['cls_id']}, cls_score={item['cls_score']:.4f}"
                )

            line_id += 1

        return results

    def draw_roi_box(self, image_bgr, box, label=''):
        x1 = int(box.x1)
        y1 = int(box.y1)
        x2 = int(box.x2)
        y2 = int(box.y2)

        cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (255, 128, 0), 2)
        if label:
            cv2.putText(
                image_bgr,
                label,
                (x1, max(0, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 128, 0),
                1,
            )

    def draw_ocr_results(self, image_bgr, line_results):
        for result in line_results:
            x1 = int(result.text_box.x1)
            y1 = int(result.text_box.y1)
            x2 = int(result.text_box.x2)
            y2 = int(result.text_box.y2)

            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'line_{result.line_id} {result.confidence:.2f}'
            cv2.putText(
                image_bgr,
                label,
                (x1, min(max(16, y1 + 16), max(16, image_bgr.shape[0] - 4))),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

    def publish_debug_image_msg(self, image_bgr, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to publish debug image: {exc}')

    def save_debug_image_file(self, image_bgr, announce):
        if not self.save_path:
            return False

        if cv2.imwrite(self.save_path, image_bgr):
            if announce:
                self.get_logger().info(f'Saved debug image to: {self.save_path}')
            return True

        self.get_logger().warn(f'Failed to save debug image to: {self.save_path}')
        return False

    def single_image_callback(self):
        if self.has_run_single_image:
            return

        self.has_run_single_image = True

        if not self.image_path:
            self.get_logger().error('image_path is empty.')
            self.request_shutdown()
            return

        if not os.path.exists(self.image_path):
            self.get_logger().error(f'image_path does not exist: {self.image_path}')
            self.request_shutdown()
            return

        image_bgr = cv2.imread(self.image_path)
        if image_bgr is None:
            self.get_logger().error(f'Failed to read image: {self.image_path}')
            self.request_shutdown()
            return

        stamp = self.get_clock().now().to_msg()
        req = self.build_full_image_request(image_bgr, 'single_image', stamp)

        try:
            line_results = self.run_ocr_lines_on_roi(
                roi=image_bgr,
                req=req,
                offset_x=0,
                offset_y=0,
            )
        except Exception as exc:
            self.get_logger().error(f'Single image OCR failed: {exc}')
            self.request_shutdown()
            return

        result_array = OcrResultArray()
        result_array.header = req.header
        result_array.results.extend(line_results)
        self.result_pub.publish(result_array)

        need_debug_image = self.publish_debug_image or bool(self.save_path)
        debug_image = image_bgr.copy() if need_debug_image else None
        if debug_image is not None:
            self.draw_roi_box(debug_image, req.box, req.target_id)
            self.draw_ocr_results(debug_image, line_results)

        if debug_image is not None and self.publish_debug_image:
            self.publish_debug_image_msg(debug_image, req.header)

        if debug_image is not None and self.save_path:
            self.save_debug_image_file(debug_image, announce=True)

        if self.log_recognized_text:
            self.get_logger().info('Single image OCR raw result:')
            for result in line_results:
                self.get_logger().info(
                    f'[line {result.line_id}] text={result.raw_text}, score={result.confidence:.4f}'
                )

            final_text = '\n'.join([result.raw_text for result in line_results if result.has_text])
            self.get_logger().info(f'Single image OCR text:\n{final_text}')
        else:
            self.get_logger().info(f'Single image OCR completed. total_lines={len(line_results)}')

        if self.exit_after_image:
            self.request_shutdown()

    def preprocess_det(self, img):
        ori_h, ori_w = img.shape[:2]
        limit_side_len = self.det_limit_side_len

        ratio = float(limit_side_len) / max(ori_h, ori_w)
        if ratio > 1.0:
            ratio = 1.0

        resize_h = int(ori_h * ratio)
        resize_w = int(ori_w * ratio)

        resize_h = max(32, int(round(resize_h / 32) * 32))
        resize_w = max(32, int(round(resize_w / 32) * 32))

        resized = cv2.resize(img, (resize_w, resize_h))

        x = resized.astype('float32') / 255.0
        x = x[:, :, ::-1]

        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        x = (x - mean) / std

        x = x.transpose(2, 0, 1)
        x = np.expand_dims(x, axis=0).astype(np.float32)

        scale_w = ori_w / float(resize_w)
        scale_h = ori_h / float(resize_h)
        return x, scale_w, scale_h

    def order_points_clockwise(self, pts):
        pts = np.array(pts, dtype=np.float32)
        rect = np.zeros((4, 2), dtype=np.float32)
        points_sum = pts.sum(axis=1)
        points_diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(points_sum)]
        rect[2] = pts[np.argmax(points_sum)]
        rect[1] = pts[np.argmin(points_diff)]
        rect[3] = pts[np.argmax(points_diff)]
        return rect

    def box_score_fast(self, bitmap, box):
        h, w = bitmap.shape[:2]
        box = box.astype(np.int32)

        xmin = int(max(0, min(np.min(box[:, 0]), w - 1)))
        xmax = int(max(0, min(np.max(box[:, 0]), w - 1)))
        ymin = int(max(0, min(np.min(box[:, 1]), h - 1)))
        ymax = int(max(0, min(np.max(box[:, 1]), h - 1)))

        if xmax <= xmin or ymax <= ymin:
            return 0.0

        mask = np.zeros((ymax - ymin + 1, xmax - xmin + 1), dtype=np.uint8)
        shifted_box = box.copy()
        shifted_box[:, 0] -= xmin
        shifted_box[:, 1] -= ymin

        cv2.fillPoly(mask, [shifted_box], 1)

        crop = bitmap[ymin:ymax + 1, xmin:xmax + 1]
        if crop.size == 0:
            return 0.0

        return float(cv2.mean(crop, mask)[0])

    def unclip_box(self, box, unclip_ratio=1.5):
        try:
            import pyclipper
            from shapely.geometry import Polygon
        except Exception as exc:
            if not self.warned_unclip_dependency:
                self.get_logger().warn(
                    'shapely and pyclipper are not installed. '
                    f'Falling back to no-unclip boxes. err={exc}'
                )
                self.warned_unclip_dependency = True
            return box

        try:
            poly = Polygon(box)
            if poly.area <= 0 or poly.length <= 0:
                return box

            distance = poly.area * unclip_ratio / poly.length

            offset = pyclipper.PyclipperOffset()
            offset.AddPath(
                box.astype(np.int32).tolist(),
                pyclipper.JT_ROUND,
                pyclipper.ET_CLOSEDPOLYGON,
            )

            expanded = offset.Execute(distance)
            if len(expanded) == 0:
                return box

            expanded = np.array(expanded[0], dtype=np.float32)
            rect = cv2.minAreaRect(expanded)
            points = cv2.boxPoints(rect)
            return self.order_points_clockwise(points)
        except Exception as exc:
            self.get_logger().warn(f'unclip_box failed. Falling back to original box. err={exc}')
            return box

    def det_postprocess(self, prob_map, scale_w, scale_h):
        bitmap = (prob_map > self.det_db_thresh).astype(np.uint8) * 255
        contours_info = cv2.findContours(
            bitmap,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        boxes = []
        max_x = max(0.0, scale_w * prob_map.shape[1] - 1.0)
        max_y = max(0.0, scale_h * prob_map.shape[0] - 1.0)

        for contour in contours:
            if contour.shape[0] < 3:
                continue

            rect = cv2.minAreaRect(contour)
            points = cv2.boxPoints(rect)
            points = self.order_points_clockwise(points)

            short_side = min(
                np.linalg.norm(points[0] - points[1]),
                np.linalg.norm(points[1] - points[2]),
            )
            if short_side < self.det_min_size:
                continue

            score = self.box_score_fast(prob_map, points)
            if score < self.det_db_box_thresh:
                continue

            points = self.unclip_box(points, unclip_ratio=self.det_db_unclip_ratio)

            rect = cv2.minAreaRect(points.astype(np.float32))
            points = cv2.boxPoints(rect)
            points = self.order_points_clockwise(points)

            points[:, 0] *= scale_w
            points[:, 1] *= scale_h
            points[:, 0] = np.clip(points[:, 0], 0.0, max_x)
            points[:, 1] = np.clip(points[:, 1], 0.0, max_y)

            box_w = np.linalg.norm(points[0] - points[1])
            box_h = np.linalg.norm(points[0] - points[3])
            if min(box_w, box_h) < self.det_min_size:
                continue

            boxes.append(points.astype(np.float32))

        boxes = self.sort_text_boxes(boxes)
        return boxes, bitmap

    def crop_rotate_image(self, img, points):
        points = self.order_points_clockwise(points)

        img_crop_width = int(
            max(
                np.linalg.norm(points[0] - points[1]),
                np.linalg.norm(points[2] - points[3]),
            )
        )
        img_crop_height = int(
            max(
                np.linalg.norm(points[0] - points[3]),
                np.linalg.norm(points[1] - points[2]),
            )
        )

        if img_crop_width <= 0 or img_crop_height <= 0:
            return None

        pts_std = np.float32([
            [0, 0],
            [img_crop_width, 0],
            [img_crop_width, img_crop_height],
            [0, img_crop_height],
        ])

        transform = cv2.getPerspectiveTransform(points.astype(np.float32), pts_std)
        dst_img = cv2.warpPerspective(
            img,
            transform,
            (img_crop_width, img_crop_height),
            borderMode=cv2.BORDER_REPLICATE,
            flags=cv2.INTER_CUBIC,
        )

        dst_h, dst_w = dst_img.shape[:2]
        if dst_w > 0 and dst_h * 1.0 / dst_w >= 1.5:
            dst_img = np.rot90(dst_img)

        return dst_img

    def add_white_padding(self, img, pad_ratio=None):
        if pad_ratio is None:
            pad_ratio = self.crop_padding_ratio

        h, w = img.shape[:2]
        pad_x = max(2, int(w * pad_ratio))
        pad_y = max(2, int(h * pad_ratio))

        return cv2.copyMakeBorder(
            img,
            pad_y,
            pad_y,
            pad_x,
            pad_x,
            cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )

    @staticmethod
    def preprocess_cls(image_bgr, cls_shape=(3, 80, 160)):
        _, image_h, image_w = cls_shape
        x = cv2.resize(image_bgr, (image_w, image_h))
        x = x.astype(np.float32) / 255.0
        x = x[:, :, ::-1]
        x = (x - 0.5) / 0.5
        x = x.transpose(2, 0, 1)
        x = np.expand_dims(x, axis=0).astype(np.float32)
        return x

    def run_cls(self, crop):
        if self.cls_sess is None:
            return crop, 0, 1.0

        x = self.preprocess_cls(crop)
        pred = self.cls_sess.run([self.cls_output_name], {self.cls_input_name: x})[0]
        cls_id = int(np.argmax(pred[0]))
        cls_score = float(np.max(pred[0]))

        # PaddleOCR 风格：只有 180 度分类置信度足够高才旋转
        if cls_id == 1 and cls_score >= self.cls_thresh:
            crop = cv2.rotate(crop, cv2.ROTATE_180)

        return crop, cls_id, cls_score

    def preprocess_rec_batch(self, crops):
        img_h = self.rec_img_h
        max_img_w = self.rec_max_img_w

        resized_items = []
        resized_widths = []

        for img in crops:
            h, w = img.shape[:2]
            if h <= 0 or w <= 0:
                continue

            ratio = w / float(h)

            if self.rec_use_dynamic_width:
                resized_w = int(np.ceil(img_h * ratio))
                resized_w = max(8, resized_w)
                resized_w = min(max_img_w, resized_w)
            else:
                resized_w = min(max_img_w, int(np.ceil(img_h * ratio)))
                resized_w = max(1, resized_w)

            x = cv2.resize(img, (resized_w, img_h))
            x = x.astype('float32') / 255.0
            x = x[:, :, ::-1]
            x = (x - 0.5) / 0.5
            x = x.transpose(2, 0, 1)

            resized_items.append(x)
            resized_widths.append(resized_w)

        if not resized_items:
            return None

        if self.rec_use_dynamic_width:
            batch_w = max(resized_widths)
            batch_w = min(max_img_w, batch_w)
        else:
            batch_w = max_img_w

        batch = np.zeros(
            (len(resized_items), 3, img_h, batch_w),
            dtype=np.float32,
        )

        for index, x in enumerate(resized_items):
            width = min(x.shape[2], batch_w)
            batch[index, :, :, :width] = x[:, :, :width]

        return batch

    def preprocess_rec(self, img):
        return self.preprocess_rec_batch([img])

    def run_rec(self, crop):
        x = self.preprocess_rec(crop)
        if x is None:
            return '', [], 0.0

        pred = self.rec_sess.run([self.rec_output_name], {self.rec_input_name: x})[0]
        indices, scores = self.ctc_decode_to_indices(pred)
        text = self.indices_to_text(indices)
        confidence = float(np.mean(scores)) if scores else 0.0
        return text, indices, confidence

    def run_rec_batch(self, crops):
        if len(crops) == 0:
            return [], [], []

        texts = []
        indices_list = []
        confs = []

        batch_size = max(1, self.rec_batch_size)
        for start_index in range(0, len(crops), batch_size):
            batch_crops = crops[start_index:start_index + batch_size]
            x = self.preprocess_rec_batch(batch_crops)
            if x is None:
                continue

            pred = self.rec_sess.run([self.rec_output_name], {self.rec_input_name: x})[0]

            for batch_index in range(pred.shape[0]):
                one_pred = pred[batch_index:batch_index + 1]
                indices, scores = self.ctc_decode_to_indices(
                    one_pred,
                    blank_index=0,
                )

                text = self.indices_to_text(indices)
                conf = float(np.mean(scores)) if scores else 0.0

                texts.append(text)
                indices_list.append(indices)
                confs.append(conf)

        return texts, indices_list, confs

    @staticmethod
    def ctc_decode_to_indices(pred, blank_index=0):
        pred = pred[0]
        raw_indices = np.argmax(pred, axis=1)
        raw_scores = np.max(pred, axis=1)

        indices = []
        scores = []
        last_idx = None

        for idx, score in zip(raw_indices, raw_scores):
            idx = int(idx)

            if idx == blank_index:
                last_idx = None
                continue

            if idx == last_idx:
                continue

            indices.append(idx)
            scores.append(float(score))
            last_idx = idx

        return indices, scores

    def indices_to_text(self, indices):
        chars = []
        for idx in indices:
            if 0 <= idx < len(self.index_to_char):
                chars.append(self.index_to_char[idx])
            else:
                chars.append('?')
        return ''.join(chars)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = OcrNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
