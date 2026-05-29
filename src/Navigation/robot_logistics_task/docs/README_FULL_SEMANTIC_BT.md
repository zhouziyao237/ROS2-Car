# Full semantic relocalization mission BT

This package implements the full robot task behavior tree:

- known current-session pose -> navigate directly
- unknown/expired pose -> exploration branch -> anchor detection -> session pose update -> navigate
- after arrival -> QR verification/refinement -> cell alignment -> platform target -> pickup done

## Main files

- `src/mission_btcpp_node.cpp`
- `behavior_tree/pickup_task_full_groot2.xml`
- `config/mission_task.yaml`
- `scripts/patch_bringup_use_full_semantic_bt.py`

## Build

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select behaviortree_cpp robot_logistics_task robot2_bringup
source install/setup.bash
```

## Patch bringup

```bash
python3 /host-home/src/Navigation/robot_logistics_task/scripts/patch_bringup_use_full_semantic_bt.py
```

## Test direct pose

```bash
ros2 topic pub --once /task/shelf_goal std_msgs/msg/String "{data: '{"task_type":"pickup","shelf_id":"TEST","row":2,"col":"B","cell_id":"TEST-L2-B","x":0.5,"y":0.0,"yaw":0.0}'}"
```

## Test unknown shelf exploration branch

```bash
ros2 topic pub --once /task/shelf_goal std_msgs/msg/String "{data: '{"task_type":"pickup","shelf_id":"S02","row":2,"col":"B","cell_id":"S02-L2-B"}'}"
```

## Real anchor input

```bash
ros2 topic pub --once /semantic/anchor_detection std_msgs/msg/String "{data: '{"shelf_id":"S02","anchor_type":"front_qr","x":1.2,"y":0.3,"yaw":0.0,"confidence":0.9}'}"
```

## Groot2

Open `behavior_tree/pickup_task_full_groot2.xml` and connect monitor to port `1667`.
