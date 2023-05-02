// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_frame.hpp>

#include "src/context.h"
#include "src/software-device.h"
#include "src/core/options.h"
#include "src/core/roi.h"
#include "src/core/extension.h"
#include "src/core/serialization.h"
#include "src/core/streaming.h"
#include "src/core/video.h"
#include "src/proc/disparity-transform.h"
#include "src/proc/synthetic-stream.h"
#include "src/media/playback/playback_device.h"
#include "src/media/playback/playback_sensor.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>

#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <bitset>
#include <string.h>

#include <atomic>

#include <dirent.h>
#include <opencv2/opencv.hpp> // Include OpenCV API

// Usefull links:
// https://github.com/IntelRealSense/librealsense/tree/master/examples/software-device
// https://github.com/IntelRealSense/librealsense/blob/master/examples/post-processing/rs-post-processing.cpp
// https://github.com/IntelRealSense/librealsense/blob/development/unit-tests/unit-tests-post-processing.cpp#L221-L241

// int main(int argc, char *argv[])
// {
//     bool _transform_to_disparity;
//     rs2::stream_profile _source_stream_profile;
//     rs2::stream_profile _target_stream_profile;
//     bool _update_target;
//     bool _stereoscopic_depth;
//     float _stereo_baseline_meter; // in meters
//     float _d2d_convert_factor;
//     size_t _width, _height;
//     size_t _bpp;
//     if (depth.get_profile().get() != _source_stream_profile.get())
//     {
//         _source_stream_profile = depth.get_profile();
//         auto info = librealsense::disparity_info::update_info_from_frame(depth);
//         _stereoscopic_depth = info.stereoscopic_depth;
//         _d2d_convert_factor = info.d2d_convert_factor;
//         auto vp = _source_stream_profile.as<rs2::video_stream_profile>();
//         _width = vp.width();
//         _height = vp.height();
//         _update_target = true;
//         auto tgt_format = _transform_to_disparity ? RS2_FORMAT_DISPARITY32 : RS2_FORMAT_Z16;
//         _target_stream_profile = _source_stream_profile.clone(RS2_STREAM_DEPTH, 0, tgt_format);
//         auto src_vspi = dynamic_cast<librealsense::video_stream_profile_interface *>(_source_stream_profile.get()->profile);
//         auto tgt_vspi = dynamic_cast<librealsense::video_stream_profile_interface *>(_target_stream_profile.get()->profile);
//         rs2_intrinsics src_intrin = src_vspi->get_intrinsics();
//         tgt_vspi->set_intrinsics([src_intrin]()
//                                  { return src_intrin; });
//         tgt_vspi->set_dims(src_intrin.width, src_intrin.height);
//     }
//     std::cout << "_stereoscopic_depth : " << _stereoscopic_depth << std::endl;
//     std::cout << "_d2d_convert_factor : " << _d2d_convert_factor << std::endl;
// }

class LocalDepthSensor
{

public:
    // [TODO: Read from calib] *************************************************
    const int W = 848;
    const int H = 480;
    const int BPP = 2;
    const float depth_unit = 0.0010000000474974513f;

    rs2_timestamp_domain domain = RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;

    rs2_intrinsics depth_intrinsics = {
        W,
        H,
        421.801025390625,
        238.7880401611328,
        421.2511291503906,
        421.2511291503906,
        RS2_DISTORTION_BROWN_CONRADY,
        {0, 0, 0, 0, 0}};
    rs2_video_stream depth_video_stream = {
        RS2_STREAM_DEPTH,
        0,
        0,
        W,
        H,
        30,
        BPP,
        RS2_FORMAT_Z16,
        depth_intrinsics};
    // ************************************************* [TODO: Read from calib]

    // Declare filters
    rs2::decimation_filter dec_filter = rs2::decimation_filter(2);        // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter = rs2::threshold_filter(0.5, 5.0);   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter = rs2::spatial_filter(0.5, 20, 2, 2); // Spatial    - spatial smoothing (alpha, delta, #filters)
    rs2::temporal_filter temp_filter = rs2::temporal_filter(0.4, 20, 3);  // Temporal   - reduces temporal noise
    rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
    rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);

    rs2::syncer sync;
    std::vector<rs2::software_sensor> depth_sensors;
    // rs2::software_sensor depth_sensor;
    rs2::stream_profile depth_stream_profile;
    rs2::frameset fset;
    rs2::frame depth;

    LocalDepthSensor(){};
    ~LocalDepthSensor(){
        // for (auto ds : depth_sensors)
        //     ds.close();
    };

    void initialize()
    {
        rs2::software_device dev;                                    // Create software-only device
        rs2::software_sensor depth_sensor = dev.add_sensor("Depth"); // Define single sensor
        depth_stream_profile = depth_sensor.add_video_stream(depth_video_stream);
        depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, depth_unit);
        depth_sensor.add_read_only_option(RS2_OPTION_STEREO_BASELINE, 50.16090393066406f);

        // dev.create_matcher(RS2_MATCHER_DLR_C);
        // sync = rs2::syncer();

        depth_sensor.open(depth_stream_profile);
        depth_sensor.start(sync);
        depth_sensors.push_back(depth_sensor);
    };

    void add_pixels(void *pixels, int frame_number)
    {
        // int frame_number = 0;
        rs2_time_t timestamp = (rs2_time_t)frame_number / 30;
        depth_sensors[0].on_video_frame({pixels,        // Frame pixels from capture API
                                         [](void *) {}, // Custom deleter (if required)
                                         W * BPP,       // Stride
                                         BPP,
                                         timestamp,
                                         domain,
                                         frame_number,
                                         depth_stream_profile,
                                         depth_unit});
        fset = sync.wait_for_frames();
        // printf("fset size : %ld\n", fset.size());
    };

    void get_depth_data()
    {
        depth = fset.first_or_default(RS2_STREAM_DEPTH);
        auto orig = dynamic_cast<librealsense::depth_frame *>((librealsense::frame_interface *)depth.get());
        auto ad = orig->additional_data;
        ad.depth_units = 0.0010000000474974513;
        orig->additional_data = ad;
    };

    rs2::frame filter_depth_data(rs2::frame depth_frame,
                                 bool decimation = false,
                                 bool threshold = true,
                                 bool spatial = true,
                                 bool temporal = true)
    {
        // // Apply filters.
        // The implemented flow of the filters pipeline is in the following order:
        // 1. apply decimation filter (downsample)
        // 2. apply threshold filter
        // 3. transform the scene into disparity domain
        // 4. apply spatial filter
        // 5. apply temporal filter
        // 6. revert the results back (if step Disparity filter was applied
        // to depth domain (each post processing block is optional and can be applied independantly).
        if (decimation)
            depth_frame = dec_filter.process(depth_frame);
        if (threshold)
            depth_frame = thr_filter.process(depth_frame);
        if (spatial || temporal)
            depth_frame = depth_to_disparity.process(depth_frame);
        if (spatial)
            depth_frame = spat_filter.process(depth_frame);
        if (temporal)
            depth_frame = temp_filter.process(depth_frame);
        if (spatial || temporal)
            depth_frame = disparity_to_depth.process(depth_frame);
        return depth_frame;
    };

    void filter_depth_data(bool decimation = false,
                           bool threshold = true,
                           bool spatial = true,
                           bool temporal = true)
    {
        depth = filter_depth_data(depth, decimation, threshold, spatial, temporal);
    };

    int view_colormap(rs2::frame depth_frame, const std::string &win_name, int interval)
    {
        rs2::video_frame vf = depth_frame.as<rs2::video_frame>();
        rs2::colorizer color_map;
        vf = color_map.process(depth_frame);
        // rs2::colorizer color_map;
        // rs2::frame depth_c = depth_frame.apply_filter(color_map);
        const int w = vf.get_width();
        const int h = vf.get_height();
        cv::Mat depth_image(cv::Size(w, h), CV_8UC3, (void *)vf.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow(win_name, depth_image);
        return (cv::waitKey(interval) & 0xFF);
    };

    int view_colormap(const std::string &win_name, int interval)
    {
        return view_colormap(depth, win_name, interval);
    };

    void save_data_to_file(const rs2::video_frame &videoframe,
                           const std::string &filepath)
    {
        stbi_write_png(filepath.c_str(),
                       videoframe.get_width(),
                       videoframe.get_height(),
                       videoframe.get_bytes_per_pixel(),
                       videoframe.get_data(),
                       videoframe.get_stride_in_bytes());
        printf("Saved : %s\n", filepath.c_str());
    };
};

int main(int argc, char *argv[])
{

    const auto depth_window_name1 = "Display Depth Image";
    cv::namedWindow(depth_window_name1, cv::WINDOW_AUTOSIZE);
    const auto depth_window_name2 = "Display Depth Image Filtered";
    cv::namedWindow(depth_window_name2, cv::WINDOW_AUTOSIZE);
    const auto depth_window_name3 = "Display Depth Image All Filtered";
    cv::namedWindow(depth_window_name3, cv::WINDOW_AUTOSIZE);
    int key1, key2, key3;

    std::string path = "/data/tmp/depth";
    // std::string path = "/data/tmp/depth_16.bin";
    // std::string path = "/code/realsense-simple-wrapper/output/testing_cpp/001622070408/1660659930/depth/000210127864.bin";
    // std::string path = "/code/realsense-simple-wrapper/data/local/realsense-15fps/001622070408/1681746140/depth/00000000000027746044.bin";

    LocalDepthSensor LDS;
    LDS.initialize();

    // Taken from: https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
    std::vector<std::string> file_names;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(path.c_str())) != NULL)
    {
        /* print all the files and directories within directory */
        while ((ent = readdir(dir)) != NULL)
        {
            if (strcmp(".", ent->d_name) == 0)
                continue;
            if (strcmp("..", ent->d_name) == 0)
                continue;
            file_names.push_back(std::string(ent->d_name));
        }
        closedir(dir);
    }
    else
    {
        /* could not open directory */
        perror("");
        return EXIT_FAILURE;
    }
    std::sort(file_names.begin(), file_names.end());

    int frame_number = 0;
    for (const std::string &file_name : file_names)
    {
        printf("%s\n", file_name.c_str());
        frame_number++;
        std::string input_path = path + "/" + file_name;
        std::ifstream input(input_path, std::ios::binary);
        char buffer[LDS.W * LDS.H * 2];
        input.read(buffer, LDS.W * LDS.H * 2);
        LDS.add_pixels(buffer, frame_number);
        LDS.get_depth_data();

        // LDS.filter_depth_data();
        key1 = LDS.view_colormap(depth_window_name1, 1);

        auto df2 = LDS.filter_depth_data(rs2::frame(LDS.depth), false, true, true, false);
        key2 = LDS.view_colormap(df2, depth_window_name2, 1);

        auto df3 = LDS.filter_depth_data(rs2::frame(LDS.depth), false, true, true, true);
        key3 = LDS.view_colormap(df3, depth_window_name3, 100);

        if (key1 == 'q' || key2 == 'q' || key3 == 'q')
            break;

        // printf("%d\n", df2 == df3);
    }
    // printf("fset size : %ld\n", LDS.sync.wait_for_frames().size());
}
