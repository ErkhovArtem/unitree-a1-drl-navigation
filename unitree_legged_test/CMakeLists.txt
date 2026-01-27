cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  unitree_legged_msgs
  std_msgs
)

catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  src/test_high_cmd_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
