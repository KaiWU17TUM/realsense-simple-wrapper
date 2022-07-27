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

    {
        argparser args(argc, argv);
        std::vector<std::string> args_list{
            "--steps",
            "--fps",
            "--height",
            "--width",
            "--color-format",
            "--depth-format",
            "--save-path",
            // "--ip",
        };
        for (auto &&arg : args_list)
        {
            if (!args.checkarg(arg))
            {
                print(arg + " is missing", 2);
                return EXIT_FAILURE;
            }
        }
        if (!args.checkarg("--ip"))
        {
            print("--ip is not used", 1);
        }
    }

    signal(SIGINT, inthand);
    stop = 0;

    try
    {
        rs2::context ctx;
        rs2wrapper rs2_dev(argc, argv, ctx);

        std::vector<std::string> available_devices_sn;
        for (auto &&available_device : rs2_dev.get_available_devices())
            available_devices_sn.push_back(available_device[0]);
        size_t num_dev = available_devices_sn.size();

        if (num_dev == 0)
            throw rs2::error("No RS device detected...");

        rs2_dev.initialize(true, true);
        rs2_dev.save_calib();
        rs2_dev.flush_frames();

        int num_zeros_to_pad = NUM_ZEROS_TO_PAD;
        size_t dev_reset_loop = 0;

        // for (auto i = 0; i < rs2_dev.fps() * rs2_dev.steps(); ++i)
        int i = 1;
        while (!stop)
        {
            std::string i_str = pad_zeros(std::to_string(i), num_zeros_to_pad);
            std::string o_str = "";

            rs2_dev.step(o_str);

            if (i % rs2_dev.fps() == 0)
                print("Step " + i_str + "   " + o_str, 0);

            if (i % (rs2_dev.fps() * 60 * 3) == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                print("Pause for 100ms ...", 1);
                rs2_dev.reset(available_devices_sn[dev_reset_loop]);
                dev_reset_loop = (dev_reset_loop + 1) % num_dev;
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