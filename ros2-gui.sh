#!/bin/bash
# =============================================================================
# ros2-gui.sh - 管理 ROS2 Docker GUI 容器（自动路径 + 简化 X11 注入）
# =============================================================================

# ---------- 自动获取脚本所在目录（支持任意路径执行） ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------- 配置（基于脚本目录自动生成，无需手动修改） ----------
CONTAINER_NAME="ros2-car"                 # 容器名称
IMAGE_NAME="my-ros2-env"                 # Docker 镜像名称
WORKSPACE_VOLUME="${SCRIPT_DIR}/ros2-car" # 容器内工作空间（映射为 /root/workspace）
HOST_MOUNT_DIR="${SCRIPT_DIR}"           # 挂载到容器的 /host-home
# -----------------------------------------------------------------

# 检查 Docker 是否可用
if ! command -v docker &> /dev/null; then
    echo "错误：未找到 docker 命令，请先安装 Docker。"
    exit 1
fi

# 允许本地 Docker 容器连接 X 服务器（静默执行）
xhost +local:docker &> /dev/null

# 显示帮助信息
show_help() {
    cat << EOF
用法: $0 {start|exec|stop|rm} [额外的 docker run 选项...]

命令:
  start   - 创建并启动持久化容器。可附加额外 docker run 参数，例如：
              $0 start --device /dev/bus/usb:/dev/bus/usb
  exec    - 进入容器，自动处理 X11 显示
  stop    - 停止容器
  rm      - 删除容器

说明:
  - 使用主机网络模式 (--net=host)
  - 挂载 X11 socket 并传递 DISPLAY 环境变量
  - 进入容器后即可运行 rviz2 等 GUI 工具
EOF
}

# 启动容器
start_container() {
    local extra_args=("$@")
    if docker inspect "$CONTAINER_NAME" &> /dev/null; then
        echo "容器 $CONTAINER_NAME 已存在。"
        if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
            echo "正在启动已停止的容器 $CONTAINER_NAME ..."
            docker start "$CONTAINER_NAME"
        else
            echo "容器 $CONTAINER_NAME 已在运行中。"
        fi
    else
        echo "正在创建并启动新容器 $CONTAINER_NAME ..."
        mkdir -p "$WORKSPACE_VOLUME"
        docker run -it -d \
            --name "$CONTAINER_NAME" \
            --net=host \
            -e DISPLAY="$DISPLAY" \
            -v "$WORKSPACE_VOLUME:/root/workspace" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
            -v "$HOST_MOUNT_DIR:/host-home" \
            --privileged \
            -v /dev:/dev \
            "${extra_args[@]}" \
            "$IMAGE_NAME" \
            bash -c "while true; do sleep 3600; done"
        echo "容器已启动。"
        echo "提示：若容器内缺少 xauth，可运行：apt update && apt install -y xauth（非必须）"
    fi
}

# 进入容器（简化版，直接传递 DISPLAY 和必要的 X11 socket）
exec_container() {
    if ! docker inspect "$CONTAINER_NAME" &> /dev/null; then
        echo "容器 $CONTAINER_NAME 不存在。请先执行 '$0 start' 创建。"
        exit 1
    fi
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
        echo "容器 $CONTAINER_NAME 未运行，正在启动..."
        docker start "$CONTAINER_NAME"
    fi

    # 可选：若存在 auto-fix-x11.sh 则执行（但忽略错误，因为 xhost 已足够）
    local AUTO_FIX_SCRIPT="${SCRIPT_DIR}/scripts/auto-fix-x11.sh"
    if [ -f "$AUTO_FIX_SCRIPT" ]; then
        echo "执行 X11 辅助脚本（忽略可能错误）..."
        "$AUTO_FIX_SCRIPT" 2>/dev/null || true
    fi

    echo "进入容器，当前 DISPLAY=$DISPLAY"
    # 直接启动 bash，DISPLAY 变量已通过 -e 传入容器
    docker exec -it -e DISPLAY="$DISPLAY" "$CONTAINER_NAME" bash
}

# 停止容器
stop_container() {
    echo "正在停止容器 $CONTAINER_NAME ..."
    docker stop "$CONTAINER_NAME"
}

# 删除容器
rm_container() {
    echo "正在删除容器 $CONTAINER_NAME ..."
    docker rm -f "$CONTAINER_NAME"
}

# 主命令解析
case "${1:-}" in
    start)
        shift
        start_container "$@"
        ;;
    exec|enter)
        exec_container
        ;;
    stop)
        stop_container
        ;;
    rm|remove)
        rm_container
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo "错误：未知命令 '$1'"
        show_help
        exit 1
        ;;
esac
