# OSRF melodic-desktop-full (plain `ros` images often lack desktop-full)
FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Unitree SDK env (see upstream README v3.2.1) — uncomment if needed
# ENV UNITREE_SDK_VERSION=3_2
# ENV UNITREE_LEGGED_SDK_PATH=/root/catkin_ws/src/unitree_ros_to_real/unitree_legged_sdk
# ENV UNITREE_PLATFORM=amd64

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    # Networking
    network-manager \
    net-tools \
    iputils-ping \
    tcpdump \
    iproute2 \
    nano \
    # Python 3
    python3-pip \
    python3-dev \
    python3-tk \
    # X11 / GL for GUI (e.g. RViz in container)
    libx11-dev \
    x11-xserver-utils \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-utils \
    ros-melodic-rviz \
    ros-melodic-tf \
    ros-melodic-tf2 \
    ros-melodic-geometry-msgs \
    ros-melodic-sensor-msgs \
    ros-melodic-nav-msgs \
    ros-melodic-visualization-msgs \
    ros-melodic-rospy \
    ros-melodic-roscpp \
    ros-melodic-message-generation \
    ros-melodic-message-runtime \
    && rm -rf /var/lib/apt/lists/*

# Pin versions for Python 3.6 compatibility
RUN python3 -m pip install --upgrade "pip < 22.0" && \
    python3 -m pip install \
    "numpy<1.20" \
    "protobuf<4.0" \
    "onnx==1.10.1" \
    "onnxruntime==1.10.0" \
    "pyyaml" \
    rospkg \
    catkin_pkg

RUN mkdir -p /root/catkin_ws/src
    # Example: git clone --recursive https://github.com/unitreerobotics/unitree_ros_to_real.git

RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Source ROS + ROS_IP helper for every shell (docker exec, new tabs)
RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ros_detect_ip.sh ] && . /ros_detect_ip.sh' >> /root/.bashrc
# Login shells may skip .bashrc — pull it in via .bash_profile
RUN touch /root/.bash_profile && \
    grep -q '/root/.bashrc' /root/.bash_profile 2>/dev/null || \
    echo '[ -f /root/.bashrc ] && . /root/.bashrc' >> /root/.bash_profile

COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY ros_detect_ip.sh /ros_detect_ip.sh
RUN chmod +x /ros_entrypoint.sh /ros_detect_ip.sh

# Default roscore URI; override at runtime if needed. ROS_IP is set by ros_detect_ip.sh when unset.
ENV ROS_MASTER_URI=http://192.168.123.12:11311

WORKDIR /root/catkin_ws

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
