cmake_minimum_required(VERSION 2.8.3)
project(occupancy_map)

find_package(catkin REQUIRED COMPONENTS
  core_msgs
  core_util
  geometry_msgs
  image_transport
  message_filters
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS core_msgs core_util geometry_msgs image_transport message_filters roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(occupancy_map src/occupancy_map.cpp)
add_dependencies(occupancy_map core_msgs_generate_messages_cpp)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (cv_bridge REQUIRED)

target_link_libraries(occupancy_map
   ${catkin_LIBRARIES} ${Eigen3_LIBS} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
