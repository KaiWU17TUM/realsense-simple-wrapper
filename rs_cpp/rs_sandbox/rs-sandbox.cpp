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

#include <iostream>
#include <fstream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <bitset>

#include <atomic>

// // https://github.com/IntelRealSense/librealsense/blob/master/examples/post-processing/rs-post-processing.cpp
// /**
// Class to encapsulate a filter alongside its options
// */
// class filter_options
// {
// public:
//     filter_options(const std::string name, rs2::filter &filter);
//     filter_options(filter_options &&other);
//     std::string filter_name;     // Friendly name of the filter
//     rs2::filter &filter;         // The filter in use
//     std::atomic_bool is_enabled; // A boolean controlled by the user that determines whether to apply the filter or not
// };

// // https://github.com/IntelRealSense/librealsense/tree/master/examples/software-device

// int main(int argc, char *argv[])
// {

//     const int W = 848;
//     const int H = 480;
//     const int BPP = 2;

//     char buffer[W * H * BPP];
//     uint16_t buffer_u16[W * H];

//     std::string path = "/code/realsense-simple-wrapper/output/testing_cpp/001622070408/1660659930/depth/000210127864.bin";
//     std::ifstream input(path, std::ios::in | std::ios::binary);
//     input.read(buffer, W * H * BPP);

//     // uint16_t result;
//     // result = (uint16_t)((((u_char)buffer[0] & 0xF) << 8) | (u_char)buffer[1]);
//     // std::cout << result << std::endl;
//     // result = (uint16_t)((((u_char)buffer[1] & 0xF) << 8) | (u_char)buffer[0]);
//     // std::cout << result << std::endl;
//     // std::bitset<16> y(result);
//     // std::cout << y << std::endl;

//     // {
//     //     auto a = (uint16_t)(u_char)buffer[1] << 8 | (uint16_t)(u_char)buffer[0];
//     //     std::bitset<16> y((uint16_t)(u_char)buffer[1] << 8 | (uint16_t)(u_char)buffer[0]);
//     //     std::cout << y << a << std::endl;
//     // }

//     // for (int i = 0; i < H * W; i + 2)
//     // {
//     //     buffer_u16[i / 2] = (uint16_t)(u_char)buffer[i + 1] << 8 | (uint16_t)(u_char)buffer[i];
//     // }

//     int frame_number = 0;
//     rs2_time_t timestamp = (rs2_time_t)16 * frame_number;
//     auto domain = RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;

//     rs2_intrinsics depth_intrinsics = {
//         848,
//         480,
//         421.801,
//         238.788,
//         421.251,
//         421.251,
//         RS2_DISTORTION_BROWN_CONRADY,
//         {0, 0, 0, 0, 0}};

//     rs2::software_device dev; // Create software-only device

//     auto depth_sensor = dev.add_sensor("Depth"); // Define single sensor

//     auto depth_stream = depth_sensor.add_video_stream(
//         {RS2_STREAM_DEPTH, 0, 0,
//          W, H, 6, BPP,
//          RS2_FORMAT_Z16, depth_intrinsics});

//     depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);
//     depth_sensor.add_read_only_option(RS2_OPTION_STEREO_BASELINE, 50.1609f);
//     // auto ds = depth_sensor.as<rs2::depth_sensor>();
//     // std::cout << ds.get_depth_scale() << std::endl;

//     rs2::syncer sync;
//     depth_sensor.open(depth_stream);
//     depth_sensor.start(sync);

//     depth_sensor.on_video_frame({buffer,        // Frame pixels from capture API
//                                  [](void *) {}, // Custom deleter (if required)
//                                  W * BPP,       // Stride
//                                  BPP,
//                                  timestamp,
//                                  domain,
//                                  frame_number,
//                                  depth_stream});

//     rs2::frameset fset = sync.wait_for_frames();
//     rs2::frame depth = fset.first_or_default(RS2_STREAM_DEPTH);

//     // Declare filters
//     rs2::decimation_filter dec_filter;                // Decimation - reduces depth frame density
//     rs2::threshold_filter thr_filter(0.1, 5.0);       // Threshold  - removes values outside recommended range
//     rs2::spatial_filter spat_filter(0.5, 25, 3.0, 2); // Spatial    - edge-preserving spatial smoothing
//     rs2::temporal_filter temp_filter;                 // Temporal   - reduces temporal noise
//     const std::string disparity_filter_name = "Disparity";
//     rs2::disparity_transform depth_to_disparity(true);
//     rs2::disparity_transform disparity_to_depth(false);

//     // std::vector<filter_options> filters;
//     // filters.emplace_back("Decimate", dec_filter);
//     // filters.emplace_back("Threshold", thr_filter);
//     // filters.emplace_back(disparity_filter_name, depth_to_disparity);
//     // filters.emplace_back("Spatial", spat_filter);
//     // filters.emplace_back("Temporal", temp_filter);

//     /* Apply filters.
//     The implemented flow of the filters pipeline is in the following order:
//     1. apply decimation filter
//     2. apply threshold filter
//     3. transform the scene into disparity domain
//     4. apply spatial filter
//     5. apply temporal filter
//     6. revert the results back (if step Disparity filter was applied
//     to depth domain (each post processing block is optional and can be applied independantly).
//     */
//     // bool revert_disparity = false;
//     // for (auto &&filter : filters)
//     // {
//     //     if (filter.is_enabled)
//     //     {
//     //         depth = filter.filter.process(depth);
//     //         if (filter.filter_name == disparity_filter_name)
//     //         {
//     //             revert_disparity = true;
//     //         }
//     //     }
//     // }
//     // if (revert_disparity)
//     // {
//     //     depth = disparity_to_depth.process(depth);
//     // }

//     // depth = dec_filter.process(depth);
//     // depth = thr_filter.process(depth);
//     // depth = depth_to_disparity.process(depth);
//     // depth = spat_filter.process(depth);
//     // depth = temp_filter.process(depth);
//     // depth = disparity_to_depth.process(depth);

//     // auto dss = rs2::depth_stereo_sensor(depth_sensor).get_depth_scale();
//     // std::cout << dss << std::endl;

//     // std::cout << depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS) << std::endl;

//     // rs2_error **e = nullptr;
//     // auto p = depth.get();
//     // auto out = rs2_get_frame_metadata(p, RS2_FRAME_METADATA_FRAME_TIMESTAMP, e);
//     // std::cout << out << std::endl;

//     auto orig = dynamic_cast<librealsense::depth_frame *>((librealsense::frame_interface *)depth.get());
//     auto ad = orig->additional_data;
//     ad.depth_units = 0.001;
//     orig->additional_data = ad;
//     std::cout << "Depth units : " << ad.depth_units << std::endl;

//     // bool _transform_to_disparity;
//     // rs2::stream_profile _source_stream_profile;
//     // rs2::stream_profile _target_stream_profile;
//     // bool _update_target;
//     // bool _stereoscopic_depth;
//     // float _stereo_baseline_meter; // in meters
//     // float _d2d_convert_factor;
//     // size_t _width, _height;
//     // size_t _bpp;

//     // if (depth.get_profile().get() != _source_stream_profile.get())
//     // {
//     //     _source_stream_profile = depth.get_profile();

//     //     auto info = librealsense::disparity_info::update_info_from_frame(depth);
//     //     _stereoscopic_depth = info.stereoscopic_depth;
//     //     _d2d_convert_factor = info.d2d_convert_factor;

//     //     auto vp = _source_stream_profile.as<rs2::video_stream_profile>();
//     //     _width = vp.width();
//     //     _height = vp.height();
//     //     _update_target = true;

//     //     auto tgt_format = _transform_to_disparity ? RS2_FORMAT_DISPARITY32 : RS2_FORMAT_Z16;
//     //     _target_stream_profile = _source_stream_profile.clone(RS2_STREAM_DEPTH, 0, tgt_format);
//     //     auto src_vspi = dynamic_cast<librealsense::video_stream_profile_interface *>(_source_stream_profile.get()->profile);
//     //     auto tgt_vspi = dynamic_cast<librealsense::video_stream_profile_interface *>(_target_stream_profile.get()->profile);
//     //     rs2_intrinsics src_intrin = src_vspi->get_intrinsics();

//     //     tgt_vspi->set_intrinsics([src_intrin]()
//     //                              { return src_intrin; });
//     //     tgt_vspi->set_dims(src_intrin.width, src_intrin.height);
//     // }
//     // std::cout << "_stereoscopic_depth : " << _stereoscopic_depth << std::endl;
//     // std::cout << "_d2d_convert_factor : " << _d2d_convert_factor << std::endl;

//     // depth = dec_filter.process(depth);
//     depth = thr_filter.process(depth);

//     // auto t = (uint16_t *)depth.get_data();
//     // t[1] = 1000;

//     // std::cout << ((uint16_t *)depth.get_data())[1] << std::endl;

//     depth = depth_to_disparity.process(depth);
//     depth = spat_filter.process(depth);
//     depth = temp_filter.process(depth);
//     depth = disparity_to_depth.process(depth);

//     // std::cout << ((float *)depth.get_data())[1] << std::endl;

//     auto vf = depth.as<rs2::video_frame>();
//     rs2::colorizer color_map;
//     vf = color_map.process(depth);

//     std::stringstream png_file;
//     png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
//     stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
//                    vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
//     std::cout << "Saved " << png_file.str() << std::endl;
// }

int main(int argc, char *argv[])
{

    // [TODO: Read from calib] *************************************************
    const int W = 848;
    const int H = 480;
    const int BPP = 2;

    int frame_number = 0;
    rs2_time_t timestamp = (rs2_time_t)16 * frame_number;
    auto domain = RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;

    rs2_intrinsics depth_intrinsics = {
        848,
        480,
        421.801,
        238.788,
        421.251,
        421.251,
        RS2_DISTORTION_BROWN_CONRADY,
        {0, 0, 0, 0, 0}};
    rs2_video_stream depth_video_stream = {
        RS2_STREAM_DEPTH,
        0,
        0,
        W,
        H,
        6,
        BPP,
        RS2_FORMAT_Z16,
        depth_intrinsics};
    // ************************************************* [TODO: Read from calib]

    std::string path = "/code/realsense-simple-wrapper/output/testing_cpp/001622070408/1660659930/depth/000210127864.bin";
    std::ifstream input(path, std::ios::in | std::ios::binary);
    char buffer[W * H * BPP];
    input.read(buffer, W * H * BPP);

    rs2::software_device dev; // Create software-only device

    auto depth_sensor = dev.add_sensor("Depth"); // Define single sensor

    auto depth_stream = depth_sensor.add_video_stream(depth_video_stream);

    depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);
    depth_sensor.add_read_only_option(RS2_OPTION_STEREO_BASELINE, 50.1609f);

    rs2::syncer sync;
    depth_sensor.open(depth_stream);
    depth_sensor.start(sync);

    depth_sensor.on_video_frame({buffer,        // Frame pixels from capture API
                                 [](void *) {}, // Custom deleter (if required)
                                 W * BPP,       // Stride
                                 BPP,
                                 timestamp,
                                 domain,
                                 frame_number,
                                 depth_stream});

    rs2::frameset fset = sync.wait_for_frames();
    rs2::frame depth = fset.first_or_default(RS2_STREAM_DEPTH);

    auto orig = dynamic_cast<librealsense::depth_frame *>((librealsense::frame_interface *)depth.get());
    auto ad = orig->additional_data;
    ad.depth_units = 0.001;
    orig->additional_data = ad;

    // Declare filters
    rs2::decimation_filter dec_filter;                // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter(0.1, 5.0);       // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter(0.5, 25, 3.0, 2); // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;                 // Temporal   - reduces temporal noise
    const std::string disparity_filter_name = "Disparity";
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    // // Apply filters.
    // The implemented flow of the filters pipeline is in the following order:
    // 1. apply decimation filter
    // 2. apply threshold filter
    // 3. transform the scene into disparity domain
    // 4. apply spatial filter
    // 5. apply temporal filter
    // 6. revert the results back (if step Disparity filter was applied
    // to depth domain (each post processing block is optional and can be applied independantly).

    depth = dec_filter.process(depth);
    depth = thr_filter.process(depth);
    depth = depth_to_disparity.process(depth);
    depth = spat_filter.process(depth);
    depth = temp_filter.process(depth);
    depth = disparity_to_depth.process(depth);

    auto vf = depth.as<rs2::video_frame>();
    rs2::colorizer color_map;
    vf = color_map.process(depth);

    std::stringstream png_file;
    png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
    stbi_write_png(png_file.str().c_str(),
                   vf.get_width(),
                   vf.get_height(),
                   vf.get_bytes_per_pixel(),
                   vf.get_data(),
                   vf.get_stride_in_bytes());
    std::cout << "Saved " << png_file.str() << std::endl;
}
