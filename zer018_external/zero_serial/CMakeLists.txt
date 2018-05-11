cmake_minimum_required(VERSION 2.8.3)
project(zero_serial)

find_package(catkin REQUIRED COMPONENTS
  core_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS core_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
