cmake_minimum_required(VERSION 2.8.3)
project(core_util)

find_package(catkin REQUIRED)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)

#install(DIRECTORY launch/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#    )

#install(TARGETS core_util
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(FILES ${PUBLIC_HEADERS}
#        DESTINATION include)

#target_include_directories(core_util INTERFACE include/)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
