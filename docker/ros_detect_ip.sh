#!/usr/bin/env bash
# Sourced from /ros_entrypoint.sh and /root/.bashrc. Sets ROS_IP when unset so topics
# advertised from the container (host network) use a reachable address, not 127.0.0.1.

unset ROS_HOSTNAME 2>/dev/null || true

_ros_master_host() {
  echo "${ROS_MASTER_URI:-http://127.0.0.1:11311}" | sed -n 's#^[^:]*://\([^:/]*\).*#\1#p'
}

_ros_pick_ipv4() {
  local host="$1" ip
  if command -v ip >/dev/null 2>&1; then
    ip="$(ip -4 route get "$host" 2>/dev/null | sed -n 's/.*[[:space:]]src[[:space:]]\([0-9.]*\).*/\1/p' | head -n1)"
    if [ -n "$ip" ]; then
      echo "$ip"
      return
    fi
    # Fallback: default-route source IP (e.g. roscore not routable yet)
    ip="$(ip -4 route get 8.8.8.8 2>/dev/null | sed -n 's/.*[[:space:]]src[[:space:]]\([0-9.]*\).*/\1/p' | head -n1)"
    if [ -n "$ip" ]; then
      echo "$ip"
      return
    fi
  fi
  if command -v hostname >/dev/null 2>&1; then
    for ip in $(hostname -I 2>/dev/null); do
      case "$ip" in 127.*|::1) continue ;; esac
      echo "$ip"
      return
    done
  fi
  echo "127.0.0.1"
}

if [ -n "${ROS_IP:-}" ]; then
  export ROS_IP
  return 0 2>/dev/null || exit 0
fi

if [ -n "${ROS_IP_OVERRIDE:-}" ]; then
  export ROS_IP="$ROS_IP_OVERRIDE"
  return 0 2>/dev/null || exit 0
fi

host="$(_ros_master_host)"
[ -z "$host" ] && host="127.0.0.1"

export ROS_IP="$(_ros_pick_ipv4 "$host")"
