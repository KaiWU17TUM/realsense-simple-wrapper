#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(rs-sandbox)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(DEPENDENCIES realsense2 realsense2-net pthread ${OpenCV_LIBS} ${DEPENDENCIES})

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES} )
include_directories(${PROJECT_NAME}
                    /usr/local/src/librealsense/third-party/tclap/include
                    /usr/local/src/librealsense/third-party
                    /usr/local/src/librealsense/common
                    /usr/local/src/librealsense
                    )

