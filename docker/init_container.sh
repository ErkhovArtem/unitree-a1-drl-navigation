#!/bin/bash
set -euo pipefail

# This runs INSIDE the container.

echo "[init] Ensuring /tmp/runtime-root exists"
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root || true

echo "[init] Ensuring nx -> 192.168.123.12 in /etc/hosts"
grep -qE '(^|\\s)nx(\\s|$)' /etc/hosts || echo "192.168.123.12 nx" >> /etc/hosts

# Image creates catkin_ws/src; compose mounts packages into src/<pkg>
mkdir -p /root/catkin_ws/src
