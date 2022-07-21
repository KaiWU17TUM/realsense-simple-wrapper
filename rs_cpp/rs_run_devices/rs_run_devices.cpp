// Based on : librealsense/examples/save-to-disk/rs-save-to-disk.cpp

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include <librealsense2/h/rs_pipeline.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <cstdlib>
#include <fstream>  // File IO
#include <iostream> // Terminal IO
#include <sstream>  // Stringstreams

#include "utils.h"
#include "rs_wrapper.h"

// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char *argv[])
try
{

    if (argc != 6 && argc != 7)
    {
        std::cerr << "Please enter fps, height, width, color format, depth format, save path, {ipaddress}" << std::endl;
        throw std::invalid_argument("There should be 6 or 7 arguments");
    }

    // Intialize the wrapper
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    // Start streaming with args defined configuration
    rs2::context ctx;
    rs2wrapper rs2_dev(argc, argv, ctx);
    rs2_dev.initialize(true, true);
    rs2_dev.save_calib();
    rs2_dev.initial_flush();

    int num_zeros_to_pad = NUM_ZEROS_TO_PAD;
    for (auto i = 0; i < rs2_dev.fps() * 10; ++i)
    {
        std::string i_str = pad_zeros(std::to_string(i), num_zeros_to_pad);
        rs2_dev.step(i_str);
        print("Step " + i_str, 0);
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling "
              << e.get_failed_function()
              << "(" << e.get_failed_args() << "):\n    "
              << e.what()
              << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
