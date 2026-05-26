# target_pickup_bringup dynamic complete

This bringup starts OAK YOLO + depth, OCR, target_matcher, target_pose_bridge, and target_auto_decision.
It also includes scripts to dynamically switch target code without keeping the previous lock.

## Install

```bash
cd /host-home/src/target_detection
rm -rf target_pickup_bringup
unzip /mnt/data/target_pickup_bringup_dynamic_complete.zip
cd /host-home/src/target_detection/target_pickup_bringup/scripts
chmod +x *.sh
```

## Patch target_matcher for dynamic target switching

```bash
cd /host-home/src/target_detection/target_pickup_bringup/scripts
./patch_target_matcher_dynamic_code.sh
```

## Build

```bash
cd /host-home
source /opt/ros/humble/setup.bash
rm -rf build/target_matcher install/target_matcher
rm -rf build/target_pickup_bringup install/target_pickup_bringup
colcon build --packages-select target_matcher target_pickup_bringup
source /host-home/install/setup.sh
```

## Start full pickup

```bash
cd /host-home/src/target_detection/target_pickup_bringup/scripts
./start_full_pickup.sh 3412 /target_matcher/matched_detection oak_camera_link
```

## Change target code

```bash
./change_target_code.sh 1731
./change_target_code.sh 3412
```

## Debug YOLO + depth only

```bash
./start_yolo_debug.sh oak_camera_link
ros2 topic echo /target_pose_base
```

## Check topics

```bash
ros2 topic echo /vision/ocr_requests
ros2 topic echo /vision/ocr_results
ros2 topic echo /target_matcher/debug_text
ros2 topic echo /target_matcher/state
ros2 topic echo /target_matcher/matched_detection yolo_msg/msg/YoloDetectionArray
ros2 topic echo /target_pose_base
```
