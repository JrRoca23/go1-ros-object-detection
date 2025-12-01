cmake_minimum_required(VERSION 3.0.2)
project(go1_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/prueba.py
  scripts/bottle_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
