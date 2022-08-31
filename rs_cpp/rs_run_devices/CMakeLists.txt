# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseWrapperRunDevices)

add_executable(rs_run_devices 
               utils.hpp 
               utils.cpp
               rs_args.hpp 
               rs_args.cpp 
               rs_utils.hpp 
               rs_utils.cpp 
               rs_wrapper.hpp 
               rs_wrapper.cpp 
               main.cpp )
set_property(TARGET rs_run_devices PROPERTY CXX_STANDARD 11)
target_link_libraries(rs_run_devices ${DEPENDENCIES} realsense2 realsense2-net pthread)
include_directories(~/librealsense/common 
                    ~/librealsense/third-party 
                    ~/librealsense/third-party/tclap/include)