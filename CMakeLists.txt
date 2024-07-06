cmake_minimum_required(VERSION 2.8.3)
project(robot_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/robot_behavior.py
  scripts/ultrasonic_sensor.py
  scripts/yolo_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
