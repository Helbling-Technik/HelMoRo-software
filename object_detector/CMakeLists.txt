cmake_minimum_required(VERSION 3.10.2)
project(object_detector)

find_package(catkin_simple REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

include_directories(/usr/local/include/opencv2)

catkin_simple()

# add files
file(GLOB_RECURSE SOURCE
  "*.h"
  "*.c"
  "*.hpp"
  "*.cpp"
)

cs_add_executable(${PROJECT_NAME} ${SOURCE})

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)

cs_install()
cs_export()
