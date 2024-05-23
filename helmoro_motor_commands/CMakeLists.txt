cmake_minimum_required(VERSION 3.10.2)
project(helmoro_motor_commands)

find_package(catkin_simple REQUIRED)


catkin_simple()

# add files
file(GLOB_RECURSE SOURCE FOLLOW_SYMLINKS
  "*.h"
  "*.c"
  "*.hpp"
  "*.cpp"
)

cs_add_executable(${PROJECT_NAME} ${SOURCE})

target_link_libraries(
  ${PROJECT_NAME}
)

cs_install()
cs_export()
