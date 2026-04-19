FROM ros:jazzy

# Auto-source ROS2 for every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Install dependencies
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
    && rm -rf /var/lib/apt/lists/*

# Install depthai
RUN pip3 install depthai --break-system-packages

WORKDIR /opt
RUN git clone https://github.com/reedhedges/AriaCoda.git /AriaCoda && cd /AriaCoda && make && make install

# Set working directory
WORKDIR /workspace

CMD ["/bin/bash"]
