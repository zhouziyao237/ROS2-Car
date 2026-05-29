# robot_logistics_task - Groot v1 / BehaviorTree.CPP v3

默认 Groot v1 端口：

```text
Publisher port: 1666
Server port: 1667
```

## 替换

```bash
cd /mnt/data
unzip robot_logistics_task_full_semantic_bt_groot1_1666_1667.zip
mv /host-home/src/Navigation/robot_logistics_task /host-home/src/Navigation/robot_logistics_task_bak_$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
cp -r robot_logistics_task /host-home/src/Navigation/
```

## 编译

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_logistics_task robot2_bringup
source install/setup.bash
```

## 接入总控

```bash
python3 /host-home/src/Navigation/robot_logistics_task/scripts/patch_bringup_use_groot1_semantic_bt.py
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot2_bringup robot_logistics_task
source install/setup.bash
```

## 单独启动

```bash
ros2 launch robot_logistics_task mission_task.launch.py
```

日志应显示：

```text
Groot v1 PublisherZMQ enabled. publisher_port=1666 server_port=1667
```

## Groot v1 连接

Server IP 填：

```text
127.0.0.1
```

然后点 connect。

## 检查端口

```bash
ss -lntup | grep -E "1666|1667|1668|1669"
```

应该只有 1666 和 1667。
