#!/usr/bin/env bash
set -e

# Source ROS Melodic base
source /opt/ros/melodic/setup.bash

# Source catkin workspace if exists
[ -f /root/catkin_ws/install/setup.bash ] && source /root/catkin_ws/install/setup.bash

# Default ROS_MASTER_URI if not set
: "${ROS_MASTER_URI:=http://192.168.123.12:11311}"
export ROS_MASTER_URI

# Keep explicit ROS_IP from the environment; otherwise ros_detect_ip.sh picks one
# shellcheck source=/dev/null
[ -f /ros_detect_ip.sh ] && . /ros_detect_ip.sh

echo "[entrypoint] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[entrypoint] ROS_IP=$ROS_IP"

exec "$@"
