# 多点导航接口说明

本文档说明 `nav_stack_core` 里当前可直接使用的多点导航接口，以及后续实现自定义多点导航功能时推荐复用的入口。

当前默认前提：

- 启动入口使用 `mapping_nav_with_serial_wheel_heading.launch.py`
- 底盘控制模式切到 `auto`
- Nav2 已经 active

推荐先执行：

```bash
source /opt/ros/humble/setup.bash
cd $WS
source install/setup.bash

ros2 launch nav_stack_core mapping_nav_with_serial_wheel_heading.launch.py
ros2 topic pub /control_mode std_msgs/msg/String "{data: auto}" -1
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses
```

## 1. 接口总览

当前建议优先使用下面两类入口：

- 单点导航：`/navigate_to_pose`
- 多点导航：`/navigate_through_poses`

另外仓库里还保留了一个文件式航点入口：

- `ros2 launch nav_stack_core waypoint_runner.launch.py`

它适合快速冒烟测试固定航点文件，但如果后面要做你自己的“多点导航功能”，更推荐你自己的节点或脚本直接调用 `/navigate_through_poses`。

## 2. 单点导航接口

动作名：

- `/navigate_to_pose`

动作类型：

- `nav2_msgs/action/NavigateToPose`

示例：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

说明：

- `frame_id` 统一使用 `map`
- 朝向必须是四元数；如果不关心朝向，可先用 `w: 1.0`
- RViz 的 `2D Goal Pose` 也是走这条链路，只是先发到 `/goal_pose`，再由 `goal_pose_bridge` 转成 `/navigate_to_pose`

## 3. 多点导航接口

动作名：

- `/navigate_through_poses`

动作类型：

- `nav2_msgs/action/NavigateThroughPoses`

示例：

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
"{poses: [
  {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}},
  {header: {frame_id: map}, pose: {position: {x: 0.0, y: 1.0, z: 0.0}, orientation: {z: 1.0, w: 0.0}}}
]}"
```

适用场景：

- 上位机一次下发多个目标点
- UI 上选中多点后一次执行
- 业务逻辑自行决定中途插点、删点、重排点序

实现建议：

- 如果你后面要做“多点导航页面”或“任务巡航节点”，建议内部统一转成 `/navigate_through_poses`
- 这样接口最直接，也最容易和你自己的任务状态机、暂停恢复、任务取消对接

## 4. 文件式航点入口

仓库里已有现成 launch：

- `src/nav_stack_core/launch/waypoint_runner.launch.py`

默认航点文件：

- `src/nav_stack_core/config/waypoints_example.yaml`

调用方式：

```bash
ros2 launch nav_stack_core waypoint_runner.launch.py
```

或指定自己的航点文件：

```bash
ros2 launch nav_stack_core waypoint_runner.launch.py \
  waypoints_file:=$WS/src/nav_stack_core/config/waypoints_example.yaml
```

这个入口更适合：

- 先验证固定航点文件是否能跑通
- 快速做实验室巡航测试
- 暂时不想自己写调用 action 的节点

## 5. 航点文件格式

参考文件：

- `src/nav_stack_core/config/waypoints_example.yaml`

格式示例：

```yaml
waypoints:
  - pose:
      header:
        frame_id: map
      pose:
        position: {x: 1.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - pose:
      header:
        frame_id: map
      pose:
        position: {x: 1.0, y: 1.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
```

要求：

- 所有航点都放在 `waypoints` 列表里
- 每个航点都使用 `PoseStamped` 结构
- `frame_id` 建议固定为 `map`
- 朝向用四元数，不要直接写欧拉角

## 6. 运行前检查

多点导航前建议先确认：

```bash
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_through_poses
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

预期：

- `/bt_navigator` 为 `active [3]`
- `/navigate_through_poses` 有 `Action servers: 1`
- `/odometry/filtered` 正常更新
- `map -> odom -> base_link` TF 连通

## 7. 当前推荐实现路线

如果后面要正式做“多点导航功能”，推荐顺序：

1. 先用命令行验证 `/navigate_through_poses`
2. 再用 `waypoint_runner.launch.py` 验证固定 YAML 航点文件
3. 最后写你自己的节点或脚本，读取业务航点并调用 `/navigate_through_poses`

当前不建议直接把多点任务逻辑塞进底盘串口节点或 launch 文件里，因为后面一旦要加暂停、恢复、跳点、重规划，维护会比较难。
