cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  core_msgs
  core_util
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp core_msgs core_util sensor_msgs std_msgs
)

add_executable(mission_manager src/mission_manager.cpp)
add_dependencies(mission_manager core_msgs_generate_messages_cpp)

include_directories (
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(mission_manager
   ${catkin_LIBRARIES}
)
