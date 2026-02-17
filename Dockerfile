# Use ROS2 Humble desktop full image as base
FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /root/workspace

# Replace Ubuntu apt sources with Tsinghua mirrors (for faster downloads in China)
RUN sed -i 's@archive.ubuntu.com@mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's@security.ubuntu.com@mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        wget \
        git \
        cmake \
        gcc \
        g++ \
        make \
        python3.8 \
        python3.8-dev \
        python3.8-venv \
        python3-pip \
        xauth \
        x11-apps \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        vim \
        curl \
        ca-certificates \
        libgl1-mesa-glx \
        libgl1-mesa-dev \
        libosmesa6-dev \
        patchelf \
    && rm -rf /var/lib/apt/lists/*

# Configure Python alternatives (optional: set python3.8 as default python3)
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1 \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Set pip index to Tsinghua mirror
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# Automatically source ROS2 environment when starting a shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ========== Install MuJoCo ==========
# Create MuJoCo directory
RUN mkdir -p /opt/mujoco

# Copy MuJoCo tarball from host (ensure the file exists in Software/ relative to build context)
COPY Software/mujoco-3.5.0-linux-x86_64.tar.gz /tmp/

# Extract and install MuJoCo, then set environment variables
RUN tar -xzf /tmp/mujoco-3.5.0-linux-x86_64.tar.gz -C /opt/mujoco --strip-components=1 && \
    rm /tmp/mujoco-3.5.0-linux-x86_64.tar.gz && \
    echo 'export MUJOCO_DIR=/opt/mujoco' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/mujoco/bin' >> /root/.bashrc && \
    echo 'export PATH=$PATH:/opt/mujoco/bin' >> /root/.bashrc

# Install MuJoCo Python bindings (mujoco-py)
RUN pip3 install mujoco-py

# ========== Copy X11 automation script ==========
# This script should handle X11 authentication checks and optionally start rviz2
COPY x11-setup.sh /root/x11-setup.sh
RUN chmod +x /root/x11-setup.sh

# Default command: run the X11 setup script
CMD ["/root/x11-setup.sh"]