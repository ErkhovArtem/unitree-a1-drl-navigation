cmake_minimum_required(VERSION 2.8.3)
project(dog_lplanner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  unitree_legged_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_msgs
)

# Устанавливаем Python скрипты
catkin_install_python(PROGRAMS
  scripts/policy_inference_ros1.py
  scripts/policy_gui.py
  scripts/inference_onnx.py
  scripts/lidar_processor_ros1.py
  scripts/lidar_sectors_node.py
  scripts/high_cmd_safe_node.py
  scripts/tf_tree_ros1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Устанавливаем launch файлы
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# Устанавливаем config файлы
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

# Устанавливаем ONNX модель (опционально)
install(FILES
  sac_actor.onnx
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
