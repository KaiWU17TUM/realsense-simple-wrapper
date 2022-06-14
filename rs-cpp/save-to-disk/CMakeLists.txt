# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesSaveToDisk)

add_executable(rs-save-to-disk rs-save-to-disk.cpp)
set_property(TARGET rs-save-to-disk PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-save-to-disk ${DEPENDENCIES} realsense2 realsense2-net)
include_directories(/usr/src/librealsense/common /usr/src/librealsense/third-party /usr/src/librealsense/third-party/tclap/include)