cmake_minimum_required(VERSION 3.10.2)
project(helmoro_joymanager)

find_package(catkin REQUIRED COMPONENTS
  helmoro_msgs
  helmoro_description
  roscpp
  sensor_msgs
  actionlib
  move_base_msgs
  any_node
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS helmoro_msgs roscpp sensor_msgs any_node helmoro_description geometry_msgs
)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/HelmoroJoyManager.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

add_dependencies(helmoro_joymanager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  param
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
