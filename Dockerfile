FROM ros:jazzy

# 1. 自动 source 环境变量
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# 2. 解决网络超时和依赖缺失问题 (开启 universe 和 multiverse 软件源)
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80retries && \
    apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository multiverse && \
    add-apt-repository restricted

# 3. 安装底盘导航、基础依赖
RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-sick-scan-xd \
    python3-pip \
    git \
    wget \
    build-essential \
    make \
    g++ \
    cmake \
    doxygen \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# 4. 安装我们刚才手动装好的 OAK-D 相机驱动、OpenCV 和 ROS 视觉桥接库
RUN apt-get update && apt-get install -y \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-usb-cam \
    ros-jazzy-depthai-ros \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 5. 安装 Python 相关的视觉库
RUN pip3 install --ignore-installed "numpy<3" depthai opencv-python --break-system-packages

# 6. 下载并编译底层底盘驱动 AriaCoda (放在最后，避免影响前面缓存)
WORKDIR /opt
RUN git clone https://github.com/reedhedges/AriaCoda.git /AriaCoda && cd /AriaCoda && make && make install

# 7. 设置工作目录
WORKDIR /workspace

CMD ["/bin/bash"]
