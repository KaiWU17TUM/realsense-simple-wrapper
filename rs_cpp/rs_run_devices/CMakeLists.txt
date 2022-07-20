# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseWrapperRunDevices)

add_executable(rs_run_devices rs_run_devices.cpp rs_wrapper.h rs_wrapper.cpp utils.h)
set_property(TARGET rs_run_devices PROPERTY CXX_STANDARD 11)
target_link_libraries(rs_run_devices ${DEPENDENCIES} realsense2 realsense2-net)
include_directories(~/librealsense/common ~/librealsense/third-party ~/librealsense/third-party/tclap/include)