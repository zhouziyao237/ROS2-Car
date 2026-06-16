# oak_guided_pickup

OAK RGB 视觉升降对齐 ROS2 包。目标包裹找到、二次货架平行对准并靠近到约 50cm 后，用目标 DataMatrix 中心点相对手动标定点 `align_u_px/align_v_px` 的像素差控制升降平台。

## 节点

```bash
ros2 run oak_guided_pickup oak_guided_pickup_node
```

## 标定工具

```bash
ros2 run oak_guided_pickup calibrate_visual_align_point
```

按键：鼠标左键设置点；滑条微调；`c` 使用当前码中心；`s` 保存 yaml；`p` 发布；`r` 重载；`q`/ESC 退出。
