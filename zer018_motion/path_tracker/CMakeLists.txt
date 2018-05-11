cmake_minimum_required(VERSION 2.8.3)
project(path_tracker)

find_package(catkin REQUIRED COMPONENTS
  core_msgs
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
