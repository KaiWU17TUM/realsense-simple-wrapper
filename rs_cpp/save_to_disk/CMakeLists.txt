# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseWrapperSaveToDisk)

add_executable(rs_save_to_disk rs_save_to_disk.cpp rs_wrapper.h rs_wrapper.cpp)
set_property(TARGET rs_save_to_disk PROPERTY CXX_STANDARD 11)
target_link_libraries(rs_save_to_disk ${DEPENDENCIES} realsense2 realsense2-net)
include_directories(/usr/src/librealsense/common /usr/src/librealsense/third-party /usr/src/librealsense/third-party/tclap/include)