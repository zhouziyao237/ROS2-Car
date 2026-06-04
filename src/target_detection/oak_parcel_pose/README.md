# oak_parcel_pose

ROS2 Python 包，用于识别 RGB 图像中的 Data Matrix，并融合 Depth 图像和 `CameraInfo` 相机内参，输出 Data Matrix 相对于相机坐标系的位置，单位为 **mm**。

## 修复说明

当前版本使用：

```python
zxingcpp.read_barcodes(img)
```

没有使用 `zxingcpp.MultiFormatReader()`，可以兼容你当前环境里的 `zxingcpp` API。

## 默认输入话题

```text
/oak/rgb/image_raw
/oak/stereo/image_raw
/oak/rgb/camera_info
```

## 默认输出话题

```text
/oak/datamatrix_result
/oak/datamatrix_3d_result_json
/oak/datamatrix_depth_debug_image
```

## 输出坐标

反投影公式：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

其中 `u, v` 是 Data Matrix 中心像素，`fx, fy, cx, cy` 来自 `/oak/rgb/camera_info`，`X/Y/Z` 全部以 **mm** 输出。

## 重要说明

深度图最好是已经对齐 RGB 的深度图。若 OAK 驱动有 aligned depth / depth_registered 话题，建议把 `depth_topic` 改成那个话题。

## 编译运行

建议先清理旧版本：

```bash
cd /host-home
rm -rf build/oak_parcel_pose install/oak_parcel_pose log
colcon build --packages-select oak_parcel_pose
source install/setup.bash
ros2 launch oak_parcel_pose oak_parcel_pose.launch.py
```

查看结果：

```bash
ros2 topic echo /oak/datamatrix_3d_result_json
```
