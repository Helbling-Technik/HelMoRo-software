cmake_minimum_required(VERSION 2.8.3)
project(helmoro_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  helmoro_msgs
  helmoro_description
  roscpp
  any_node
  robot_state_publisher
  kdl_parser
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    helmoro_msgs 
    roscpp 
    any_node 
    helmoro_description
    robot_state_publisher
    kdl_parser
    nav_msgs
    tf
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
  src/HelmoroStatePublisher.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

add_dependencies(helmoro_state_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
