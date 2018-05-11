cmake_minimum_required(VERSION 2.8.3)
project(zero_monitor)

find_package(catkin REQUIRED COMPONENTS
  core_msgs
  roscpp
  sensor_msgs
  std_msgs
  image_transport
)

catkin_package(
#  LIBRARIES zero_monitor
  CATKIN_DEPENDS core_msgs roscpp sensor_msgs image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/zero_monitor.cpp)

add_executable(zero_monitor src/zero_monitor.cpp)
add_dependencies(zero_monitor core_msgs_generate_messages_cpp)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (cv_bridge REQUIRED)

target_link_libraries(zero_monitor
   ${catkin_LIBRARIES} ${Eigen3_LIBS} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
