# ROS2 GUI in Docker with X11 Forwarding (via MobaXterm)

本项目提供了一套脚本和配置，让你能够在 **Windows 主机（使用 MobaXterm）** 上通过 SSH 连接到 **Ubuntu 22.04 服务器**，并在 **Docker 容器** 中运行 ROS2 图形界面程序（如 `rviz2`、`gazebo`），窗口直接显示在 Windows 桌面上。

## 特性

- 容器长期运行，工作区数据通过 Docker 卷持久化。
- 每次 SSH 登录后自动适应变化的 `DISPLAY` 变量。
- 支持 OpenGL 应用（可启用软件渲染）。
- 包含 MuJoCo 物理引擎安装（可选）。
- 核心脚本位于 `~/ros2-car/`：
  - `scripts/auto-fix-x11.sh`：宿主机自动修复 X11 认证。
  - `ros2-gui.sh`：管理持久化容器。
  - `x11-setup.sh`：容器内 X11 环境检查。
  - `Dockerfile`：构建自定义镜像。

## 系统要求

- **服务器**：Ubuntu 22.04，已安装 Docker 和 `xauth`。
- **客户端**：Windows 系统，安装 [MobaXterm](https://mobaxterm.mobatek.net/)（内置 X 服务器）。
- **网络**：SSH 连接已启用 X11 转发（MobaXterm 会话中勾选 **X11-Forwarding**）。

## 快速开始

### 1. 在服务器上准备环境

```bash
sudo apt update
sudo apt install xauth x11-apps -y
xhost +local:docker
```

### 2. 部署脚本

将本项目所有文件放置于 `~/ros2-car`，目录结构如下：
```
~/ros2-car/
├── Dockerfile
├── ros2-gui.sh
├── x11-setup.sh
├── scripts/
│   └── auto-fix-x11.sh
└── Software/                # 可选，存放 MuJoCo 安装包
    └── mujoco-3.5.0-linux-x86_64.tar.gz
```

赋予执行权限：
```bash
chmod +x ~/ros2-car/ros2-gui.sh ~/ros2-car/scripts/auto-fix-x11.sh
```

将自动修复脚本加入 `~/.bashrc`：
```bash
echo "~/ros2-car/scripts/auto-fix-x11.sh" >> ~/.bashrc
```

### 3. 构建镜像

```bash
cd ~/ros2-car
docker build -t my-ros2 .
```
> 如不需要 MuJoCo，请编辑 Dockerfile 删除相关部分。

### 4. 管理容器

- **启动容器**：`./ros2-gui.sh start`
- **进入容器**：`./ros2-gui.sh exec`
- **停止容器**：`./ros2-gui.sh stop`
- **删除容器**：`./ros2-gui.sh rm`

### 5. 测试 GUI

进入容器后运行：
```bash
xeyes                 # 应弹出窗口
source /opt/ros/humble/setup.bash
rviz2                 # 启动 RViz2
gazebo                # 启动 Gazebo
```

若 RViz2 报 OpenGL 错误，启用软件渲染：
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
rviz2
```

## 故障排除

### 认证失败（`Authorisation not recognised`）

1. **检查宿主机认证**：
   ```bash
   xauth list $DISPLAY
   ```
   若为空，请重新用 `ssh -Y` 连接。

2. **确保 `localhost` 条目存在**：
   ```bash
   xauth list | grep localhost
   ```
   如缺失，手动运行 `~/ros2-car/scripts/auto-fix-x11.sh` 或添加：
   ```bash
   xauth add localhost:显示编号 MIT-MAGIC-COOKIE-1 <cookie值>
   ```

3. **重建认证文件**（当添加失败时）：
   ```bash
   ./ros2-gui.sh stop
   mv ~/.Xauthority ~/.Xauthority.old
   touch ~/.Xauthority && chmod 600 ~/.Xauthority
   xauth add $DISPLAY MIT-MAGIC-COOKIE-1 $(xauth list $DISPLAY | awk '{print $3}')
   DISPLAY_NUM=$(echo $DISPLAY | sed -n 's/.*:\([0-9]*\)\..*/\1/p')
   xauth add localhost:$DISPLAY_NUM MIT-MAGIC-COOKIE-1 $(xauth list $DISPLAY | awk '{print $3}')
   ./ros2-gui.sh start
   ./ros2-gui.sh exec
   xeyes
   ```

4. **临时绕过认证**（仅测试）：
   ```bash
   xhost +
   # 修改 ros2-gui.sh 中 docker exec 行为：-e DISPLAY=":10" -e XAUTHORITY=""
   ./ros2-gui.sh exec
   ```

### RViz2 启动失败（OpenGL）
使用软件渲染环境变量。

### Python 版本注意事项
默认 Python 3.10，如需 Python 3.8 请用 `python3.8` 命令，勿改动系统默认。

## 文件说明

| 文件 | 位置 | 作用 |
|------|------|------|
| `scripts/auto-fix-x11.sh` | `~/ros2-car/scripts/` | 为 `localhost`/`127.0.0.1` 添加当前显示编号的 X11 cookie。 |
| `ros2-gui.sh` | `~/ros2-car/` | 管理容器的启动、进入、停止、删除。 |
| `Dockerfile` | `~/ros2-car/` | 构建包含 ROS2、Gazebo、MuJoCo 的镜像。 |
| `x11-setup.sh` | 容器内 `/root/x11-setup.sh` | 容器启动时检查 X11 环境并可选启动 RViz2。 |
| `Software/mujoco-*.tar.gz` | `~/ros2-car/Software/`（需自行下载） | MuJoCo 安装包，可省略。 |

## 许可证

本项目脚本仅供个人学习使用，可根据需要自由修改。ROS2 和相关软件遵循其原始许可证。
