cmake_minimum_required(VERSION 2.8.3)
project(helmoro_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xacro
  urdf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    xacro
    urdf
    roscpp
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  meshes
  urdf
  launch
  param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
