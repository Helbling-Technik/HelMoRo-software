cmake_minimum_required(VERSION 3.10.2)
project(helmoro_2dnav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ##astra_camera
  move_base
  helmoro_state_publisher
  ##rplidar_ros
)