#!/bin/bash

# ===================================================
# X11 Forwarding 自动化配置和检查脚本
# 版本: 1.0
# 描述: 自动检查并配置 X11 转发环境，测试图形界面
# ===================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   X11 转发环境配置和检查脚本 v1.0    ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# ========== 步骤1: 检查必需工具 ==========
echo -e "${YELLOW}[1/8] 检查必需工具...${NC}"

# 检查 xauth
if ! command -v xauth &> /dev/null; then
    echo -e "  ?? xauth 未安装，正在安装..."
    apt update && apt install -y xauth
else
    echo -e "  ? xauth 已安装: $(which xauth)"
fi

# 检查 x11-apps
if ! command -v xeyes &> /dev/null; then
    echo -e "  ?? x11-apps 未安装，正在安装..."
    apt install -y x11-apps
else
    echo -e "  ? x11-apps 已安装"
fi

# ========== 步骤2: 检查挂载的认证文件 ==========
echo -e "${YELLOW}[2/8] 检查挂载的认证文件...${NC}"

if [ -f /tmp/.docker.xauth ]; then
    echo -e "  ? 认证文件已挂载: /tmp/.docker.xauth"
    ls -la /tmp/.docker.xauth
else
    echo -e "  ? 错误: 认证文件未挂载!"
    echo -e "     请确保运行容器时挂载了认证文件:"
    echo -e "     --volume=\"\$HOME/docker-x11/.docker.xauth:/tmp/.docker.xauth:rw\""
    exit 1
fi

# ========== 步骤3: 检查认证文件权限 ==========
echo -e "${YELLOW}[3/8] 检查认证文件权限...${NC}"

if [ -w /tmp/.docker.xauth ]; then
    echo -e "  ? 认证文件可写"
else
    echo -e "  ?? 认证文件不可写，尝试修复权限..."
    chmod 666 /tmp/.docker.xauth
    if [ $? -eq 0 ]; then
        echo -e "  ? 权限修复成功"
    else
        echo -e "  ? 权限修复失败，请检查挂载方式"
    fi
fi

# ========== 步骤4: 读取认证信息 ==========
echo -e "${YELLOW}[4/8] 读取认证信息...${NC}"

XAUTH_INFO=$(xauth -f /tmp/.docker.xauth list 2>&1)
if [ $? -eq 0 ] && [ -n "$XAUTH_INFO" ]; then
    echo -e "  ? 认证信息有效:"
    echo -e "  $XAUTH_INFO"
else
    echo -e "  ? 认证文件无效或为空"
    echo -e "  错误信息: $XAUTH_INFO"
    exit 1
fi

# ========== 步骤5: 设置环境变量 ==========
echo -e "${YELLOW}[5/8] 设置环境变量...${NC}"

export XAUTHORITY=/tmp/.docker.xauth
# export DISPLAY=localhost:11.0

echo -e "  ? XAUTHORITY=$XAUTHORITY"
echo -e "  ? DISPLAY=$DISPLAY"

# ========== 步骤6: 验证环境变量 ==========
echo -e "${YELLOW}[6/8] 验证环境变量...${NC}"

# 检查 DISPLAY 格式
if [[ "$DISPLAY" =~ ^localhost:[0-9]+(\.[0-9]+)?$ ]] || [[ "$DISPLAY" =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
    echo -e "  ? DISPLAY 格式正确"
else
    echo -e "  ?? DISPLAY 格式可能有问题: $DISPLAY"
fi

# 检查 XAUTHORITY
if [ -f "$XAUTHORITY" ]; then
    echo -e "  ? XAUTHORITY 指向有效文件"
else
    echo -e "  ? XAUTHORITY 指向的文件不存在"
    exit 1
fi

# ========== 步骤7: 测试 X11 转发 ==========
echo -e "${YELLOW}[7/8] 测试 X11 转发 (xeyes)...${NC}"

echo -e "  正在启动 xeyes (3秒内应该出现窗口)..."
xeyes &
EYES_PID=$!

# 等待3秒看进程是否还在运行
sleep 3
if kill -0 $EYES_PID 2>/dev/null; then
    echo -e "  ? xeyes 运行成功! X11 转发正常"
    kill $EYES_PID 2>/dev/null
else
    echo -e "  ? xeyes 启动失败，X11 转发可能有问题"
    echo -e "  尝试手动运行: xeyes"
fi

# ========== 步骤8: 提供 RViz2 启动命令 ==========
echo -e "${YELLOW}[8/8] 准备启动 RViz2...${NC}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}? 配置完成! 可以启动 RViz2${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "运行以下命令启动 RViz2:"
echo -e "  ${BLUE}source /opt/ros/humble/setup.bash${NC}"
echo -e "  ${BLUE}rviz2${NC}"
echo ""
echo -e "如果遇到问题，可以手动测试:"
echo -e "  ${BLUE}xeyes${NC}  # 测试基本 X11 转发"
echo -e "  ${BLUE}xauth list${NC}  # 查看认证信息"
echo -e "  ${BLUE}echo \$DISPLAY${NC}  # 查看显示设置"
echo ""

# 询问是否自动启动 RViz2
read -p "是否现在启动 RViz2? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "启动 RViz2..."
    source /opt/ros/humble/setup.bash
    rviz2
fi

# 保持容器运行
exec /bin/bash