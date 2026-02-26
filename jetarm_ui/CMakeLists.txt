cmake_minimum_required(VERSION 3.0.2)
project(jetarm_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib
  hiwonder_interfaces
  dynamic_reconfigure
  astra_camera
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs actionlib hiwonder_interfaces dynamic_reconfigure astra_camera
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
