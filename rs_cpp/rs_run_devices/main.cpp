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
    std::map<std::string, storagepath> storagepaths,
    std::string device_sn,
    size_t num_devices,
    std::chrono::steady_clock::time_point global_timestamp)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50 * (th_id + 1)));
    rs2args rs2_arg = rs2args(argc, argv);

    // if limit is given the stream needs to reset for it to take effect.
    {
        // Initializing RS devices in parallel can be problematic with libusb.
        std::lock_guard<std::mutex> guard(mux);
        rs2wrapper _rs2_dev(rs2_arg, false, context, device_sn);
        _rs2_dev.initialize_depth_sensor_ae();
    }

    rs2wrapper rs2_dev(rs2_arg, context, device_sn);
    rs2_dev.set_storagepaths(storagepaths);

    {
        // Initializing RS devices in parallel can be problematic with libusb.
        std::lock_guard<std::mutex> guard(mux);
        rs2_dev.initialize(true);
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
        std::vector<std::vector<std::string>> device_list;
        std::map<std::string, storagepath> storagepaths;

        {
            rs2wrapper _rs2_dev = rs2wrapper(argc, argv, false, ctx, "-1");
            device_list = _rs2_dev.get_available_devices();
            _rs2_dev.prepare_storage();
            storagepaths = _rs2_dev.get_storagepaths();
        }

        if (device_list.size() == 0)
            throw rs2::error("No RS device detected...");

        std::chrono::steady_clock::time_point global_timestamp = std::chrono::steady_clock::now();

        size_t num_threads = device_list.size();
        std::vector<std::thread> threads;
        for (size_t i = 0; i < num_threads; ++i)
            threads.push_back(
                std::thread([=]
                            { multithreading_function(i,
                                                      argc,
                                                      argv,
                                                      ctx,
                                                      storagepaths,
                                                      device_list[i][0],
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
        rs2args rs2_arg = rs2args(argc, argv);

        // if limit is given the stream needs to reset for it to take effect.
        if (rs2_arg.depth_sensor_autoexposure_limit() !=
            rs2_arg.default_depth_sensor_autoexposure_limit)
        {
            rs2wrapper _rs2_dev(rs2_arg, false, ctx, "-1");
            _rs2_dev.initialize_depth_sensor_ae();
        }

        rs2wrapper rs2_dev(rs2_arg, ctx, "-1");

        auto available_devices = rs2_dev.get_available_devices();
        if (available_devices.size() == 0)
            throw rs2::error("No RS device detected...");

        rs2_dev.prepare_storage();
        rs2_dev.initialize(true);
        rs2_dev.save_calib();
        rs2_dev.flush_frames();

        size_t dev_reset_loop = 0;
        size_t num_dev = rs2_dev.get_enabled_devices().size();

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
                rs2_dev.reset(available_devices[dev_reset_loop][0]);
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

    if (args.check_rs2args() == EXIT_FAILURE)
        return EXIT_FAILURE;

    args.print_args();

    if (args.multithreading())
        return run_multithreading(argc, argv);
    else
        return run(argc, argv);
}
