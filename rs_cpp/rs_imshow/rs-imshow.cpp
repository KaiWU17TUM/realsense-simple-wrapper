// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// #include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// #include <opencv2/opencv.hpp>   // Include OpenCV API

#include <iostream>

#define CL_HPP_TARGET_OPENCL_VERSION 210
#include <CL/cl2.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;

int main(int argc, char * argv[]) try
{
    char env[]="OPENCV_OPENCL_DEVICE=Intel:CPU:"; 
    // char env[]="OPENCV_OPENCL_DEVICE=:GPU:"; 
    putenv(env);     
    setUseOptimized(true);
    printf("OpenCL Found : %s\n", cv::ocl::haveOpenCL() ? "true" : "false");
    printf("Using OpenCL : %s\n", cv::ocl::useOpenCL() ? "true" : "false");

    rs2::context ctx;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe(ctx);

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);

    // Start streaming with default recommended configuration
    pipe.start(cfg);

    const auto depth_window_name1 = "Display Depth1 Image";
    const auto color_window_name1 = "Display Color1 Image";
    namedWindow(depth_window_name1, WINDOW_AUTOSIZE);
    namedWindow(color_window_name1, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && 
           getWindowProperty(depth_window_name1, WND_PROP_AUTOSIZE) >= 0 && 
           getWindowProperty(color_window_name1, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame color = data.get_color_frame();

        // Query frame size (width and height)
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat depth_image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        Mat color_image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(depth_window_name1, depth_image);
        imshow(color_window_name1, color_image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


// int main(int argc, char * argv[])
// {

//     std::vector<cl::Platform> platforms;
//     cl::Platform::get(&platforms);
//     for (int j = 0; j < platforms.size(); j++) {
//         auto p = platforms[j];//Change platform from 0,1 and 2
//         std::vector <cl::Device> devices;
//         p.getDevices(CL_DEVICE_TYPE_ALL, &devices);
//         for (int i = 0; i < devices.size(); i++) {
//             auto device = devices[i];
//             std::cout << device.getInfo<CL_DEVICE_VENDOR>() << std::endl;
//             std::cout << device.getInfo<CL_DEVICE_VERSION>() << std::endl;
//             std::cout << device.getInfo<CL_DEVICE_TYPE>() << std::endl;
//             std::cout << CL_DEVICE_TYPE_CPU << std::endl;
//             std::cout << CL_DEVICE_TYPE_GPU << std::endl;
//             std::cout << CL_DEVICE_TYPE_ACCELERATOR << std::endl;
//             std::cout << CL_DEVICE_TYPE_DEFAULT << std::endl;
//             std::cout << device.getInfo<CL_DRIVER_VERSION>() << std::endl;
//             // std::cout << device.getInfo<CL_DEVICE_TYPE_GPU>() << std::endl;
//             // std::cout << device.getInfo<CL_DEVICE_TYPE_ACCELERATOR>() << std::endl;
//             // std::cout << device.getInfo<CL_DEVICE_TYPE_DEFAULT>() << std::endl;
//         }
//         std::cout << "----------------------\n";
//     }

//     return 1;
// }