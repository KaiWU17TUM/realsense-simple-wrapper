#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSenseKinfuExample)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenCL REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR} ${OpenCL_INCLUDE_DIR} )

set(DEPENDENCIES realsense2 pthread glfw ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${OpenCL_LIBRARIES} ${DEPENDENCIES})

add_executable(rs-kinfu rs-kinfu.cpp /usr/local/src/librealsense/examples/example.hpp)
set_property(TARGET rs-kinfu PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-kinfu ${DEPENDENCIES})
include_directories(/usr/local/src/librealsense/examples)
