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
#include "utils.hpp"
#include "rs_wrapper.hpp"

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
        std::vector<std::string> REQUIRED_ARGS{
            "--steps",
            "--fps",
            "--height",
            "--width",
            "--color-format",
            "--depth-format",
            "--save-path",
        };
        std::vector<std::string> OPTIONAL_ARGS{
            "--reset-interval",
            "--flush-steps",
            "--ip",
        };
        for (auto &&arg : REQUIRED_ARGS)
        {
            if (!args.checkarg(arg))
            {
                print(arg + " is missing", 2);
                return EXIT_FAILURE;
            }
        }
        for (auto &&arg : OPTIONAL_ARGS)
            if (!args.checkarg(arg))
                print(arg + " is not used", 1);
    }

    signal(SIGINT, inthand);
    stop = 0;

    try
    {
        rs2::context ctx;
        rs2wrapper rs2_dev(argc, argv, ctx);
        rs2args rs2_arg = rs2_dev.get_args();

        std::vector<std::string> available_devices_sn = rs2_dev.get_available_devices_sn();
        if (available_devices_sn.size() == 0)
            throw rs2::error("No RS device detected...");

        rs2_dev.initialize(true, true);
        rs2_dev.save_calib();
        rs2_dev.flush_frames(rs2_arg.flush_steps());

        int num_zeros_to_pad = 16;
        int dev_reset_loop = 0;
        std::vector<std::string> enabled_devices_sn = rs2_dev.get_enabled_devices_sn();

        rs2_dev.reset_global_timestamp();

        int i = 1;
        while (!stop)
        {
            std::string i_str = pad_zeros(std::to_string(i), num_zeros_to_pad);
            std::string o_str = "";

            rs2_dev.step();
            o_str += rs2_dev.get_output_msg();

            if (i % rs2_arg.fps() == 0)
                print("Step " + i_str + "   " + o_str, 0);

            if (i % rs2_arg.reset_interval() == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                print("Sleep for 100ms ...", 1);
                rs2_dev.reset(available_devices_sn[dev_reset_loop]);
                dev_reset_loop = (dev_reset_loop + 1) % enabled_devices_sn.size();
            }

            if (i >= rs2_arg.steps())
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