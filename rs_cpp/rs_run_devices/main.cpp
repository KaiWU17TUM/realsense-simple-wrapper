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
#include <mutex>

// #include <tclap/CmdLine.h>
#include "utils.hpp"
#include "rs_wrapper.hpp"

volatile sig_atomic_t stop = 0;
const int num_zeros_to_pad = 16;
std::mutex mux;

void inthand(int signum)
{
    stop = 1;
    print("ctrl + c detected", 1);
}

void multithreading_function(
    size_t th_id,
    int argc,
    char *argv[],
    rs2::context context,
    std::string device_sn,
    size_t num_devices,
    std::chrono::steady_clock::time_point global_timestamp)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50 * (th_id + 1)));

    rs2wrapper rs2_dev(argc, argv, context, device_sn);
    rs2args rs2_arg = rs2_dev.get_args();

    {
        // Initializing RS devices in parallel can be problematic with libusb.
        std::lock_guard<std::mutex> guard(mux);
        rs2_dev.initialize(true, true);
        rs2_dev.save_calib();
        rs2_dev.flush_frames();
    }

    rs2_dev.reset_global_timestamp(global_timestamp);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int i = 1;
    int i_offset = rs2_arg.reset_interval() * th_id;
    int i_range = rs2_arg.reset_interval() * num_devices;
    while (!stop)
    {
        std::string i_str = pad_zeros(std::to_string(i), num_zeros_to_pad);
        std::string o_str = "";

        rs2_dev.step();
        o_str += rs2_dev.get_output_msg();

        if (i % rs2_arg.fps() == 0)
            print("Step " + i_str + "   " + o_str, 0);

        if ((i + i_offset) % i_range == 0)
            rs2_dev.reset(device_sn);

        if (i >= rs2_arg.steps())
            break;
        else
            i++;
    }

    rs2_dev.stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool run_multithreading(int argc, char *argv[])
{
    try
    {
        rs2::context ctx;
        std::vector<std::string> device_sn_list;

        {
            rs2wrapper rs2_dev = rs2wrapper(argc, argv, ctx);
            device_sn_list = rs2_dev.get_available_devices_sn();
        }

        if (device_sn_list.size() == 0)
            throw rs2::error("No RS device detected...");

        std::chrono::steady_clock::time_point global_timestamp = std::chrono::steady_clock::now();

        size_t num_threads = device_sn_list.size();
        std::vector<std::thread> threads;
        for (size_t i = 0; i < num_threads; ++i)
            threads.push_back(
                std::thread([=]
                            { multithreading_function(i,
                                                      argc,
                                                      argv,
                                                      ctx,
                                                      device_sn_list[i],
                                                      num_threads,
                                                      global_timestamp); }));

        for (int i = 0; i < num_threads; ++i)
        {
            threads[i].join();
            std::cout << "joining t" << i << std::endl;
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
}

bool run(int argc, char *argv[])
{
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
        rs2_dev.flush_frames();

        size_t dev_reset_loop = 0;
        size_t num_dev = rs2_dev.get_enabled_devices_sn().size();

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
                rs2_dev.reset(available_devices_sn[dev_reset_loop]);
                dev_reset_loop = (dev_reset_loop + 1) % num_dev;
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

int main(int argc, char *argv[])
{
    signal(SIGINT, inthand);

    rs2args args(argc, argv);
    auto valid_args = args.check_rs2args();
    if (valid_args == EXIT_FAILURE)
        return EXIT_FAILURE;

    if (args.checkarg("--multithreading"))
        return run_multithreading(argc, argv);
    else
        return run(argc, argv);
}
