#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSenseImShowExample)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

set(DEPENDENCIES realsense2 pthread ${OpenCV_LIBS} ${DEPENDENCIES})

add_executable(rs-imshow rs-imshow.cpp)
set_property(TARGET rs-imshow PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-imshow ${DEPENDENCIES})