#!/bin/bash
# Filename: ros2-gui.sh
# Purpose: Manage a persistent ROS2 GUI container with automatic X11 forwarding fix.
#          Designed to work with host's auto-fix-x11.sh script.

# ========== Configuration (修改为你的实际值) ==========
CONTAINER_NAME="ros2-gui"          # 容器名称
IMAGE_NAME="my-ros2"               # 你的镜像名称
WORKSPACE_VOLUME="ros2_workspace"   # 数据卷名称，用于持久化工作区
# =====================================================

# 检查 Docker 是否可用
if ! command -v docker &> /dev/null; then
    echo "Error: docker command not found. Please install Docker first."
    exit 1
fi

# 每次执行脚本时确保宿主机允许 Docker 容器连接 X 服务器（只需一次）
xhost +local:docker &> /dev/null

# 显示帮助信息
show_help() {
    cat << EOF
Usage: $0 {start|exec|stop|rm}

Commands:
  start   - Create and start the persistent container (if exists, start it)
  exec    - Enter the container, automatically fix X11 auth and pass DISPLAY
  stop    - Stop the container
  rm      - Remove the container

Description:
  - Uses host networking (--net=host) so container shares host's network.
  - Mounts host's .Xauthority and X11 socket for GUI forwarding.
  - Before entering, runs host's ~/ros2-car/scripts/auto-fix-x11.sh to ensure correct X11 cookies.
  - Forces DISPLAY to "localhost:display_number" inside container for compatibility.
EOF
}

# 启动容器（如果不存在则创建，已存在则启动）
start_container() {
    if docker inspect "$CONTAINER_NAME" &> /dev/null; then
        echo "Container $CONTAINER_NAME already exists."
        if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
            echo "Starting stopped container $CONTAINER_NAME..."
            docker start "$CONTAINER_NAME"
        else
            echo "Container $CONTAINER_NAME is already running."
        fi
    else
        echo "Creating and starting new container $CONTAINER_NAME ..."
        # 创建数据卷（如果不存在）
        docker volume inspect "$WORKSPACE_VOLUME" &> /dev/null || docker volume create "$WORKSPACE_VOLUME"
        # 运行容器（后台，长期运行）
        docker run -it -d \
            --name "$CONTAINER_NAME" \
            --net=host \
            -v "$WORKSPACE_VOLUME:/root/workspace" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
            "$IMAGE_NAME" \
            bash -c "while true; do sleep 3600; done"
        echo "Container started."
    fi
}

# 进入容器（自动修复 X11 并传递环境变量）
exec_container() {
    if ! docker inspect "$CONTAINER_NAME" &> /dev/null; then
        echo "Container $CONTAINER_NAME does not exist. Run '$0 start' first."
        exit 1
    fi
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
        echo "Container $CONTAINER_NAME is not running. Starting it..."
        docker start "$CONTAINER_NAME"
    fi

    # 第一步：在宿主机上运行 auto-fix-x11.sh（如果存在），确保 .Xauthority 包含 localhost 条目
    if [ -f "$HOME/ros2-car/scripts/auto-fix-x11.sh" ]; then
        echo "Updating host X11 authentication with auto-fix-x11.sh..."
        "$HOME/ros2-car/scripts/auto-fix-x11.sh"
    else
        echo "Warning: $HOME/ros2-car/scripts/auto-fix-x11.sh not found. Make sure .Xauthority has correct entries."
    fi

    # 第二步：从当前 DISPLAY 中提取显示编号（例如从 ros2-server:10.0 提取 10）
    DISPLAY_NUM=$(echo "$DISPLAY" | sed -n 's/.*:\([0-9]*\)\..*/\1/p')
    if [ -z "$DISPLAY_NUM" ]; then
        echo "Error: Cannot extract display number from DISPLAY='$DISPLAY'"
        exit 1
    fi

    # 第三步：使用 localhost 作为显示服务器地址，并明确传递 XAUTHORITY
    echo "Entering container $CONTAINER_NAME with DISPLAY=localhost:$DISPLAY_NUM"
    docker exec -it \
        -e DISPLAY="localhost:$DISPLAY_NUM" \
        -e XAUTHORITY=/root/.Xauthority \
        "$CONTAINER_NAME" \
        bash
}

# 停止容器
stop_container() {
    echo "Stopping container $CONTAINER_NAME ..."
    docker stop "$CONTAINER_NAME"
}

# 删除容器
rm_container() {
    echo "Removing container $CONTAINER_NAME ..."
    docker rm -f "$CONTAINER_NAME"
}

# 主命令解析
case "${1:-}" in
    start)
        start_container
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
        echo "Error: unknown command '$1'"
        show_help
        exit 1
        ;;
esac