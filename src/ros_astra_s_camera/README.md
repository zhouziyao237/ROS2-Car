# ros_astra_s_camera_full_ws

This workspace contains two ROS2 Humble packages for an Orbbec Astra S depth camera:

- `ros_astra_s_camera_msgs`: custom messages for target pixel input and millimeter 3D position output.
- `ros_astra_s_camera`: Astra S OpenNI2 camera launcher, RGB-D fusion localization node, depth calibration node, and RViz2 config.

The package is designed for the Astra S device that appears as:

```bash
lsusb
# ID 2bc5:0402 ORBBEC ASTRA S
```

It uses the OpenNI2 SDK path that worked in the current setup:

```bash
/host-home/Software/OpenNi2/sdk/libs
/host-home/Software/OpenNi2/sdk/libs/OpenNI2/Drivers
```

You can override these paths using environment variables:

```bash
export OPENNI2_REDIST=/your/OpenNi2/sdk/libs
export OPENNI2_DRIVERS_PATH=/your/OpenNi2/sdk/libs/OpenNI2/Drivers
```

---

## 1. Copy packages into your workspace

If you unzip this file under `/host-home`:

```bash
cd /host-home
unzip ros_astra_s_camera_full_ws.zip
cp -r ros_astra_s_camera_full_ws/src/* /host-home/src/
```

---

## 2. Dependencies

You need the OpenNI2 camera package already built or available in the workspace:

```bash
openni2_camera
```

You also need these ROS packages:

```bash
apt update
apt install -y \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-rviz2 \
  ros-humble-camera-calibration
```

The Astra S must be visible by USB:

```bash
lsusb | grep -i orbbec
```

Expected:

```bash
Bus XXX Device XXX: ID 2bc5:0402 ORBBEC ASTRA S
```

Recommended USB permission command:

```bash
chmod -R 777 /dev/bus/usb
```

---

## 3. Build

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash  # optional, if you already have other packages built

colcon build --packages-select ros_astra_s_camera_msgs ros_astra_s_camera
source install/setup.bash
```

---

## 4. Run camera and RGB-D localization

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py
```

Successful Astra S recognition should show:

```bash
Device "2bc5/0402@..." found.
Starting color stream.
Starting depth stream.
```

The calibration warning is acceptable for initial testing:

```bash
Camera calibration file /root/.ros/camera_info/rgb_Astra_Orbbec.yaml not found
```

The driver will still publish a default `CameraInfo`.

---

## 5. Run with RViz2

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_rviz:=true
```

Or open RViz2 manually:

```bash
rviz2 -d /host-home/install/ros_astra_s_camera/share/ros_astra_s_camera/rviz/astra_s_camera.rviz
```

RViz2 displays:

- `/camera/rgb/image_raw`
- `/camera/depth/image`
- `/astra_s/debug_rgbd_image`
- `/astra_s/target_point`
- TF frames

If the fixed frame has an error, set RViz2 `Fixed Frame` to one of:

```bash
camera_link
camera_rgb_optical_frame
camera_depth_optical_frame
```

---

## 6. Run with point cloud

The normal launch uses `camera_only.launch.py`. To use `camera_with_cloud.launch.py`:

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_cloud:=true use_rviz:=true
```

Then check cloud topics:

```bash
ros2 topic list | grep -E "points|cloud"
```

---

## 7. Topics

Camera topics are from `openni2_camera`, commonly:

```bash
/camera/rgb/image_raw
/camera/depth/image
/camera/rgb/camera_info
/camera/depth/camera_info
```

This package adds:

```bash
/astra_s/target_pixel          # ros_astra_s_camera_msgs/msg/TargetPixel input
/astra_s/target_position_mm    # ros_astra_s_camera_msgs/msg/TargetPositionMM output, unit mm
/astra_s/target_point          # geometry_msgs/msg/PointStamped output, unit m, for RViz2
/astra_s/debug_rgbd_image      # debug image with target and XYZ text
```

---

## 8. Message definitions

### TargetPixel.msg

```text
std_msgs/Header header
float64 u
float64 v
float64 confidence
string target_id
```

Use this when your detection package has a bounding box center:

```text
u = bbox_center_x
v = bbox_center_y
```

### TargetPositionMM.msg

```text
std_msgs/Header header
string target_id
float64 x_mm
float64 y_mm
float64 z_mm
float64 u
float64 v
float64 depth_mm
float64 confidence
bool valid
```

The 3D position is computed as:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

All `TargetPositionMM` coordinates are in millimeters.

---

## 9. Test RGB-D fusion

Publish a target pixel at the image center:

```bash
ros2 topic pub --once /astra_s/target_pixel ros_astra_s_camera_msgs/msg/TargetPixel "{u: 320.0, v: 240.0, confidence: 1.0, target_id: 'center'}"
```

Check millimeter output:

```bash
ros2 topic echo /astra_s/target_position_mm
```

Check RViz2 point output in meters:

```bash
ros2 topic echo /astra_s/target_point
```

---

## 10. Calibration mode

Run:

```bash
ros2 launch ros_astra_s_camera astra_s_calibration.launch.py
```

This starts:

1. Astra S camera driver.
2. ROS2 `camera_calibration` for RGB camera calibration.
3. `depth_calibration_node.py` for center-depth size checking.

The checkerboard settings are in:

```bash
ros_astra_s_camera/config/astra_s_camera.yaml
```

Default:

```yaml
calibration:
  board_size: "8x11"
  square_size_m: "0.030"
```

`board_size` means internal corner count, not square count.

---

## 11. Depth size calibration

The depth node prints center median raw depth in millimeters:

```bash
Center median raw depth = 850.0 mm
```

Measure the real physical distance with a ruler and fit:

```text
real_depth_mm = raw_depth_mm * depth_scale_correction + depth_offset_mm
```

Then update:

```yaml
rgbd_position_node:
  ros__parameters:
    depth_scale_correction: 1.0
    depth_offset_mm: 0.0
```

---

## 12. RGB-to-depth pixel alignment

If your detector gives RGB pixel `(u, v)` but the depth point is slightly shifted, tune:

```yaml
rgb_to_depth_offset_u: 0.0
rgb_to_depth_offset_v: 0.0
rgb_to_depth_scale_u: 1.0
rgb_to_depth_scale_v: 1.0
```

Mapping formula:

```text
u_depth = u_rgb * rgb_to_depth_scale_u + rgb_to_depth_offset_u
v_depth = v_rgb * rgb_to_depth_scale_v + rgb_to_depth_offset_v
```

---

## 13. Integration with another detection package

Your detection package should publish:

```bash
/astra_s/target_pixel
```

Message type:

```bash
ros_astra_s_camera_msgs/msg/TargetPixel
```

Example Python publisher logic:

```python
from ros_astra_s_camera_msgs.msg import TargetPixel

msg = TargetPixel()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "camera_rgb_optical_frame"
msg.u = float(bbox_center_x)
msg.v = float(bbox_center_y)
msg.confidence = float(confidence)
msg.target_id = "object"
publisher.publish(msg)
```

Then subscribe to:

```bash
/astra_s/target_position_mm
```

or RViz2-friendly:

```bash
/astra_s/target_point
```

---

## 14. Troubleshooting

### OpenNI2 finds 0 devices

Make sure the launch uses the Orbbec SDK through LD_PRELOAD. This package sets it automatically through launch environment variables. Check paths in:

```bash
ros_astra_s_camera/config/astra_s_camera.yaml
```

Also confirm:

```bash
find /host-home/Software/OpenNi2/sdk/libs/OpenNI2/Drivers -type f
```

Expected:

```bash
liborbbec.so
libOniFile.so
orbbec.ini
OniFile.ini
```

### System OpenNI2 is used by mistake

Check:

```bash
ldd /host-home/install/openni2_camera/lib/libopenni2_camera_lib.so | grep -i openni
```

If it points to `/lib/x86_64-linux-gnu/libOpenNI2.so.0`, the launch `LD_PRELOAD` is required. This package sets it automatically.

### No depth image

Check:

```bash
ros2 topic list | grep depth
ros2 topic hz /camera/depth/image
```

You may try cloud mode:

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_cloud:=true
```
