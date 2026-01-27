#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test node for publishing fixed HighCmd messages.
This node publishes a fixed HighCmd message for testing purposes.
Compatible with Python 3.6 and ROS Melodic (Ubuntu 18.04).
"""

import rospy
from unitree_legged_msgs.msg import HighCmd, LED


def create_test_high_cmd():
    """
    Create a fixed HighCmd message for testing.
    
    Returns:
        HighCmd: A HighCmd message with fixed test values
    """
    msg = HighCmd()
    
    # Basic parameters
    msg.levelFlag = 0x00
    # msg.commVersion = 0x01
    # msg.robotID = 0x00
    # msg.SN = 0x00000000
    # msg.bandWidth = 0x00
    msg.mode = 0x01  # 0: idle, 2: force stand, 6: trotting
    
    # Movement parameters
    msg.forwardSpeed = 0.0
    msg.sideSpeed = 0.0
    msg.rotateSpeed = 0.0
    msg.bodyHeight = 0.0
    msg.footRaiseHeight = 0.0
    
    # Orientation
    msg.yaw = 0.0
    msg.pitch = 0.5
    msg.roll = 0.0
    
    # LED array (4 LEDs)
    msg.led = [LED() for _ in range(4)]
    for i in range(4):
        msg.led[i].r = 0
        msg.led[i].g = 0
        msg.led[i].b = 0
    
    # Remote control data
    msg.wirelessRemote = [0] * 40
    # msg.AppRemote = [0] * 40
    # msg.reserve = 0
    msg.crc = 0
    
    return msg


def main():
    """
    Main function to publish HighCmd messages.
    """
    # Initialize ROS node
    rospy.init_node('test_high_cmd_publisher', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=10)
    
    # Get publishing rate from parameter, default 10 Hz
    rate = rospy.Rate(rospy.get_param('~rate', 10.0))
    
    # Create the test message
    test_msg = create_test_high_cmd()
    
    rospy.loginfo("Starting test HighCmd publisher...")
    rospy.loginfo("Publishing to topic: /high_cmd")
    rospy.loginfo("Publishing rate: {} Hz".format(rospy.get_param('~rate', 10.0)))
    
    # Publish messages
    while not rospy.is_shutdown():
        # Log message details periodically (every 5 seconds at 10 Hz = every 50 messages)
        if rospy.get_time() % 5.0 < 0.1:
            rospy.loginfo("Publishing HighCmd: mode={}, forwardSpeed={}, sideSpeed={}, rotateSpeed={}".format(
                test_msg.mode, test_msg.forwardSpeed, test_msg.sideSpeed, test_msg.rotateSpeed))
        
        pub.publish(test_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
