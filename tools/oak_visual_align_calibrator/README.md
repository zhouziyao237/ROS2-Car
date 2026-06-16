# OAK RGB 对齐点标定工具

路径建议安装到：

```text
/host-home/tools/oak_visual_align_calibrator
```

启动：

```bash
bash /host-home/tools/oak_visual_align_calibrator/run_visual_align_calibrator.sh
```

功能：

- 显示 `/oak/rgb/image_raw` 实时画面。
- 绿色十字为 `align_u_px / align_v_px`。
- 红点为当前目标 DataMatrix 中心点。
- 鼠标左键点击画面可设置对齐点。
- 滑条可微调 `align_u_px / align_v_px`。
- `c`：使用当前 DataMatrix 中心点作为对齐点。
- `s`：保存到 `/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml`。
- `r`：从 yaml 重新读取默认点。
- `p`：只发布当前点，不保存。
- `q` 或 `ESC`：退出。

默认对齐点：

```yaml
align_u_px: 38.0
align_v_px: 100.0
```

这个点只给 `oak_guided_pickup` 内部使用，不影响 TF、URDF、Nav2、oak_package_search、oak_package_servo 或 DataMatrix 检测。
