# qr_platform_lift_control

Incremental lift controller for QR/OAK target pose.

## What it does

- Subscribes to `/target_pose_base` (`geometry_msgs/msg/PointStamped`, unit m)
- Uses `oak_camera_link` coordinates:
  - `x`: forward distance, used only for validity filtering
  - `y`: lateral offset, used only for validity filtering
  - `z`: vertical offset, used for lift control
- Compensates optional camera pitch.
- Filters invalid depth jumps.
- Publishes incremental lift target to `/platform/target`.

## Install

Copy this folder to:

```bash
/host-home/src/Navigation/qr_platform_lift_control
```

Build:

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select qr_platform_lift_control --symlink-install
source /host-home/install/setup.bash
```

## Run

Start QR/depth pipeline first, with decision disabled:

```bash
ros2 launch target_pickup_bringup qr_pickup_pipeline.launch.py \
  target_code:=4309 \
  target_frame:=oak_camera_link \
  start_decision:=false
```

Start lift controller:

```bash
ros2 launch qr_platform_lift_control qr_lift_control.launch.py \
  initial_lift_target_mm:=300.0 \
  camera_above_platform_mm:=100.0 \
  camera_pitch_deg:=0.0 \
  lift_direction:=1.0
```

If lift moves in the wrong direction:

```bash
lift_direction:=-1.0
```

If camera is pitched upward, try:

```bash
camera_pitch_deg:=8.0
```

or opposite sign:

```bash
camera_pitch_deg:=-8.0
```

## Debug

```bash
ros2 topic echo /qr_lift_control/debug
ros2 topic echo /target_pose_base
ros2 topic echo /platform/target
```

## Important

Check your platform target message type:

```bash
ros2 topic type /platform/target
ros2 interface show $(ros2 topic type /platform/target)
```

If it is not `rm_serial_driver/msg/PlatformTarget`, launch with:

```bash
platform_target_type:=your_pkg/msg/YourMsg
```

If field names differ, edit `launch/qr_lift_control.launch.py`:

```python
'lift_field': 'target_lift_z',
'slide_field': 'target_slide_x',
'screw_field': 'target_screw_x',
```
