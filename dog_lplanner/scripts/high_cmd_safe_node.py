#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Safe bridge for Unitree HighCmd.

Subscribes:
- ~in_topic (default: /high_cmd_safe)  unitree_legged_msgs/HighCmd

Publishes:
- ~out_topic (default: /high_cmd)      unitree_legged_msgs/HighCmd

Logic:
- If /policy_safe_mode == true OR /policy_running == false -> publish STOP command
- Else -> forward latest received command

This allows the policy to always publish to /high_cmd_safe, while the robot only
receives commands when Safe mode is OFF.
"""

import rospy
from unitree_legged_msgs.msg import HighCmd


PARAM_RUNNING = "/policy_running"
PARAM_SAFE_MODE = "/policy_safe_mode"


def make_stop_cmd() -> HighCmd:
    cmd = HighCmd()
    # A1/Aliengo format
    cmd.levelFlag = 0xEE
    cmd.commVersion = 0
    cmd.robotID = 0
    cmd.SN = 0
    cmd.bandWidth = 0
    cmd.mode = 0  # IDLE
    cmd.forwardSpeed = 0.0
    cmd.sideSpeed = 0.0
    cmd.rotateSpeed = 0.0
    cmd.bodyHeight = 0.0
    cmd.footRaiseHeight = 0.0
    cmd.yaw = 0.0
    cmd.pitch = 0.0
    cmd.roll = 0.0
    cmd.reserve = 0
    cmd.crc = 0
    return cmd


class HighCmdSafeNode:
    def __init__(self) -> None:
        rospy.init_node("high_cmd_safe", anonymous=True)

        self.in_topic = rospy.get_param("~in_topic", "/high_cmd_safe")
        self.out_topic = rospy.get_param("~out_topic", "/high_cmd")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.timeout_s = float(rospy.get_param("~timeout", 0.25))

        if not rospy.has_param(PARAM_RUNNING):
            rospy.set_param(PARAM_RUNNING, False)
        if not rospy.has_param(PARAM_SAFE_MODE):
            rospy.set_param(PARAM_SAFE_MODE, True)

        self.pub = rospy.Publisher(self.out_topic, HighCmd, queue_size=1)
        self._last_msg = None
        self._last_msg_time = None

        self.sub = rospy.Subscriber(self.in_topic, HighCmd, self._cb, queue_size=1)

        rospy.loginfo("HighCmdSafeNode initialized")
        rospy.loginfo("  in_topic: %s", self.in_topic)
        rospy.loginfo("  out_topic: %s", self.out_topic)
        rospy.loginfo("  rate: %.1f Hz, timeout: %.2fs", self.rate_hz, self.timeout_s)

    def _cb(self, msg: HighCmd) -> None:
        self._last_msg = msg
        self._last_msg_time = rospy.Time.now()

    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        stop = make_stop_cmd()

        while not rospy.is_shutdown():
            running = bool(rospy.get_param(PARAM_RUNNING, False))
            safe = bool(rospy.get_param(PARAM_SAFE_MODE, True))

            if (not running) or safe:
                self.pub.publish(stop)
                rate.sleep()
                continue

            # Forward latest, but if stale -> stop
            now = rospy.Time.now()
            if self._last_msg is None or self._last_msg_time is None:
                self.pub.publish(stop)
            else:
                age = (now - self._last_msg_time).to_sec()
                if age > self.timeout_s:
                    self.pub.publish(stop)
                else:
                    self.pub.publish(self._last_msg)

            rate.sleep()


def main() -> None:
    try:
        node = HighCmdSafeNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

