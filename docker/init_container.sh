#!/bin/bash
set -euo pipefail

# This runs INSIDE the container.

echo "[init] Ensuring /tmp/runtime-root exists"
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root || true

echo "[init] Ensuring nx -> 192.168.123.12 in /etc/hosts"
grep -qE '(^|\\s)nx(\\s|$)' /etc/hosts || echo "192.168.123.12 nx" >> /etc/hosts

# Initialize catkin workspace
mkdir -p /root/catkin_ws/src

# Create symlinks for packages
if [ -d /workspace/unitree_legged_msgs ]; then
  echo "[init] Linking unitree_legged_msgs into catkin_ws"
  ln -sfn /workspace/unitree_legged_msgs /root/catkin_ws/src/unitree_legged_msgs
else
  echo "[init] /workspace/unitree_legged_msgs not found; skipping"
fi

if [ -d /workspace/dog_lplanner ]; then
  echo "[init] Linking dog_lplanner into catkin_ws"
  ln -sfn /workspace/dog_lplanner /root/catkin_ws/src/dog_lplanner
else
  echo "[init] /workspace/dog_lplanner not found; skipping"
fi

if [ -d /workspace/unitree_legged_test ]; then
  echo "[init] Linking unitree_legged_test into catkin_ws"
  ln -sfn /workspace/unitree_legged_test /root/catkin_ws/src/unitree_legged_test
else
  echo "[init] /workspace/unitree_legged_test not found; skipping"
fi

echo "[init] Done. Packages are linked but not built."
echo "[init] Build manually with: cd /root/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make install"

