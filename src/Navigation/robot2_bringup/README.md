# robot2_bringup

YAML-driven CyberDog-style bringup package for robot2.

## Main command

```bash
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task
```

## Modes

- `hardware`: URDF + lidar + chassis serial
- `mapping`: lidar + scan_filter + rf2o + slam
- `nav`: mapping + chassis + Nav2 + goal bridge
- `nav_d435_light`: nav + D435 depth scan
- `task`: nav + semantic shelf registrar
- `task_d435`: task + D435 QR/depth layer
- `debug`: RViz only

## Add new feature

Do not edit `yaml_bringup.launch.py`.

Add a module under `config/bringup_modes.yaml`:

```yaml
oak_yolo:
  type: node
  package: oak_vision_driver
  executable: oak_vision_driver_node
  name: oak_yolo_node
  delay: 30.0
  respawn: true
  respawn_delay: 2.0
  parameters:
    - config_file: /host-home/src/Perception/oak_vision_driver/config/yolov5_oak.yaml
```

Then add it to a mode:

```yaml
task_oak:
  enable:
    - robot_description
    - lidar
    - front_scan_filter
    - rf2o
    - rf2o_odom_adapter
    - slam
    - nav2
    - oak_yolo
    - semantic_registrar
```
