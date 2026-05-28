# Important current change

`nav_stack_core_v2` should no longer be the full robot launch entry.

Old launch files such as:

- `robot2_d435_fused_slam_semantic.launch.py`
- `d435_assist.launch.py`
- `d435_usb2_assist.launch.py`
- `semantic_shelf.launch.py`

can remain for backup/debugging, but daily startup should use:

```bash
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task
```

The YAML config calls nodes from:

- `nav_stack_core_v2`
- `lslidar_driver`
- `rm_serial_driver`
- `slam_toolbox`
- `nav2_bringup`
- `robot_logistics_semantic`

This keeps the architecture layered and prevents the launch file from becoming a large monolith again.
