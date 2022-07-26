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

#include <unistd.h>
#include <stdio.h>
#include <signal.h>

// #include <tclap/CmdLine.h>
#include "utils.h"
#include "rs_wrapper.h"

volatile sig_atomic_t stop;

void inthand(int signum)
{
    stop = 1;
    print("ctrl + c detected", 1);
}

int main(int argc, char *argv[])
{

    if (argc != 7 && argc != 8)
    {
        std::cerr << "Please enter steps, fps, height, width, color format, depth format, save path, {ipaddress}" << std::endl;
        std::cerr << "There should be 7 or 8 arguments" << std::endl;
        return EXIT_FAILURE;
    }

    signal(SIGINT, inthand);
    stop = 0;

    try
    {
        rs2::context ctx;
        rs2wrapper rs2_dev(argc, argv, ctx);

        if (rs2_dev.get_available_devices().size() == 0)
            throw rs2::error("No RS device detected...");

        rs2_dev.initialize(true, true);
        rs2_dev.save_calib();
        rs2_dev.flush_frames();

        int num_zeros_to_pad = NUM_ZEROS_TO_PAD;
        // for (auto i = 0; i < rs2_dev.fps() * rs2_dev.steps(); ++i)
        int i = 0;
        while (!stop)
        {
            std::string i_str = pad_zeros(std::to_string(i), num_zeros_to_pad);
            std::string o_str = "";

            rs2_dev.step(o_str);

            if (i % rs2_dev.fps() == 0)
                print("Step " + i_str + "   " + o_str, 0);

            if (i % (rs2_dev.fps() * 60 * 3) == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                print("Pause for 500ms ...", 1);
            }

            if (i >= rs2_dev.fps() * rs2_dev.steps())
                break;
            else
                i++;
        }
        rs2_dev.stop();
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
}