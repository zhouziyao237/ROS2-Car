# OCR sync workspace

This workspace contains:

- `target_msgs/yolo_msg`: YOLO detection messages.
- `target_msgs/ocr_msg`: OCR request/result messages.
- `ocr_node`: PP-OCRv5 ONNX INT8 OCR extraction node with timestamp-based RGB image cache.

## OCR node responsibility

`ocr_node` only extracts raw OCR text. It does not do regex matching, target-code matching, YOLO-OCR fusion, or depth fusion.

Flow:

```text
/camera/rgb/image_raw
      ↓ cached by timestamp
/vision/ocr_requests  ocr_msg/msg/OcrRequestArray
      ↓ find RGB image by request header.stamp
crop ROI by Box2D
      ↓
PP-OCRv5 det → cls → rec → CTC decode
      ↓
/vision/ocr_results  ocr_msg/msg/OcrResultArray
```

## Timestamp sync

The node caches recent RGB frames by `Image.header.stamp`. When an OCR request arrives, it tries:

1. Exact stamp match.
2. Nearest cached image if enabled.
3. Drop request if nearest image exceeds `max_image_time_diff_ms`.

Parameters:

```yaml
image_cache_size: 10
max_image_time_diff_ms: 80.0
use_nearest_image_if_exact_stamp_missing: true
ocr_queue_size: 8
```

## Build

```bash
cd /host-home/src
unzip /mnt/data/ocr_sync_ws.zip

cd /host-home/src/ocr_sync_ws/ocr_node
cp /mnt/PaddleOCR/onnx-int8/det_static_qdq_int8.onnx weights/
cp /mnt/PaddleOCR/onnx-int8/cls_static_qdq_int8.onnx weights/
cp /mnt/PaddleOCR/onnx-int8/rec_static_qdq_int8.onnx weights/
cp /mnt/PaddleOCR/models/PP-OCRv5_server_rec_infer/inference.yml weights/

cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_msg ocr_msg ocr_node
source install/setup.bash
```

## Run

```bash
ros2 launch ocr_node ocr.launch.py
```

## Test request

Make sure the camera image topic is running. Then publish an OCR request with a timestamp close to the current image stream:

```bash
ros2 run ocr_node test_pub_request.py
```

Or manually:

```bash
ros2 topic pub --once /vision/ocr_requests ocr_msg/msg/OcrRequestArray "{
  header: {frame_id: 'camera_rgb_optical_frame'},
  requests: [
    {
      target_id: 'manual_0',
      class_name: 'manual',
      yolo_confidence: 1.0,
      need_ocr: true,
      box: {x1: 0, y1: 0, x2: 640, y2: 480, center_u: 320, center_v: 240, width: 640, height: 480}
    }
  ]
}"
```

View results:

```bash
ros2 topic echo /vision/ocr_results
rqt_image_view /vision/ocr_debug_image
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/msg/Image` | sub |
| `/vision/ocr_requests` | `ocr_msg/msg/OcrRequestArray` | sub |
| `/vision/ocr_results` | `ocr_msg/msg/OcrResultArray` | pub |
| `/vision/ocr_debug_image` | `sensor_msgs/msg/Image` | pub |
