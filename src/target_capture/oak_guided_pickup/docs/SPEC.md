# oak_guided_pickup 重构 SPEC

## 核心流程

```text
ReceiveShelfGoal -> QueryShelfPose -> NavigateToShelf -> WaitNavArrive -> WaitNavSettle
-> RequestOakPackageSearchAlign -> WaitAfterOakAlign
-> ScanTargetPackageOnShelf -> CacheFoundPackagePose
-> RequestOakPackageSearchAlign -> WaitAfterOakAlign   # 二次货架平行对准，必须保留
-> RequestOakServoApproach                            # 靠近到约 50cm
-> ExecuteOakGuidedPickup                             # 升降视觉对齐
-> PublishTaskDone
```

二次 `RequestOakPackageSearchAlign` 不是重新搜索包裹，而是让车体与货架正面保持平行，再执行后续靠近和升降对齐。

## oak_guided_pickup 职责

目标包裹已经找到、目标已缓存、车体二次平行对准并靠近 50cm 后，使用目标 DataMatrix 在 OAK RGB 图像中的中心点，与手动标定的 `align_u_px/align_v_px` 计算像素误差。第一版只用 v 方向控制升降：

```text
error_v_px = code_v_px - align_v_px
desired_lift_z_mm = actual_lift_z_mm + lift_pixel_direction * kv_to_lift_mm_per_px * error_v_px
```

`layer_id` 只用于任务信息和日志，不做固定层高映射。

## rm_serial_driver 模式要求

手动/自动切换必须走 rm_serial_driver 已有链路。节点读取 `/chassis_control_mode`，manual/unknown 时只同步当前平台高度，不发布视觉控制目标。manual -> auto 时必须先同步 actual height，再 hold，再限速闭环，不能突增/突降。

## 标定工具

```bash
ros2 run oak_guided_pickup calibrate_visual_align_point
```

支持鼠标点击 OAK RGB 图像中的点、滑条微调 x/y、按 `c` 使用当前 DataMatrix 中心、按 `s` 保存到 yaml。

## 禁止

禁止删除二次平行对准；禁止把二次对准变成重新搜索/左右扫；禁止把 `RequestOakServoCenterAlign` / `RequestOakServoFineAlign` 作为硬依赖；禁止使用 Float32 平台接口；禁止绕过 rm_serial_driver；禁止修改 URDF/TF/Nav2/DataMatrix 检测核心/rm_serial 协议。
