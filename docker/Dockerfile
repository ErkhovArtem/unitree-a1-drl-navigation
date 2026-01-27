# Используем образ ROS Melodic Desktop Full от OSRF (в официальном 'ros' обычно нет desktop-full)
FROM osrf/ros:melodic-desktop-full

# Отключаем интерактивные запросы при установке пакетов
ENV DEBIAN_FRONTEND=noninteractive

# Переменные окружения для Unitree SDK (согласно README v3.2.1)
# ENV UNITREE_SDK_VERSION=3_2
# ENV UNITREE_LEGGED_SDK_PATH=/root/catkin_ws/src/unitree_ros_to_real/unitree_legged_sdk
# ENV UNITREE_PLATFORM=amd64

# Обновляем систему и устанавливаем системные утилиты, инструменты сети и ROS пакеты
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    # Сетевые инструменты
    network-manager \
    net-tools \
    iputils-ping \
    tcpdump \
    iproute2 \
    # Текстовый редактор
    nano \
    # Python 3 и pip
    python3-pip \
    python3-dev \
    python3-tk \
    # X11 и OpenGL библиотеки для GUI приложений (в т.ч. rviz в контейнере)
    libx11-dev \
    x11-xserver-utils \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-utils \
    # ROS пакеты
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

# Установка Python пакетов через pip3
# Для Python 3.6 необходимо фиксировать версии пакетов, так как новые версии их не поддерживают
RUN python3 -m pip install --upgrade "pip < 22.0" && \
    python3 -m pip install \
    "numpy<1.20" \
    "protobuf<4.0" \
    "onnx==1.10.1" \
    "onnxruntime==1.10.0" \
    "pyyaml" \
    rospkg \
    catkin_pkg

# Создание catkin workspace и установка unitree_ros_to_real
RUN mkdir -p /root/catkin_ws/src
    # Клонируем основной репозиторий со всеми субмодулями (SDK уже внутри как субмодуль)
    # git clone --recursive https://github.com/unitreerobotics/unitree_ros_to_real.git

# Создаем директорию для XDG_RUNTIME_DIR
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Чтобы в каждом термине (docker exec, новые вкладки) уже был source ROS
RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc

# Копируем и настраиваем entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Настройка переменных окружения ROS (дефолтные значения)
ENV ROS_MASTER_URI=http://192.168.123.12:11311
ENV ROS_IP=

# Настройка рабочей директории
WORKDIR /root/catkin_ws

ENTRYPOINT ["/ros_entrypoint.sh"]

# Команда по умолчанию
CMD ["bash"]
