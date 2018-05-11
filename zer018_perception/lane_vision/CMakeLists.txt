cmake_minimum_required(VERSION 2.8.3)
project(lane_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs core_msgs
#  DEPENDS system_lib
)
