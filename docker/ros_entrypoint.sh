#!/usr/bin/env bash
set -e

# Source ROS Melodic base
source /opt/ros/melodic/setup.bash

# Source catkin workspace if exists
[ -f /root/catkin_ws/install/setup.bash ] && source /root/catkin_ws/install/setup.bash

# Default ROS_MASTER_URI if not set
: "${ROS_MASTER_URI:=http://192.168.123.12:11311}"
export ROS_MASTER_URI

# Always unset ROS_HOSTNAME to avoid conflicts with ROS_IP
unset ROS_HOSTNAME

# Calculate ROS_IP based on the route to ROS_MASTER_URI host
MASTER_HOST="$(echo "$ROS_MASTER_URI" | sed -n 's#^http://\([^:/]*\).*#\1#p')"
if command -v ip >/dev/null 2>&1 && [ -n "$MASTER_HOST" ]; then
  # Try to get the IP address of the interface used to reach the master
  DETECTED_IP="$(ip route get "$MASTER_HOST" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -n1 || true)"
  if [ -n "$DETECTED_IP" ]; then
    export ROS_IP="$DETECTED_IP"
  fi
fi

# Fallback/Override logic
export ROS_IP="${ROS_IP:-${ROS_IP_OVERRIDE:-}}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

echo "[entrypoint] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[entrypoint] ROS_IP=$ROS_IP"

exec "$@"
