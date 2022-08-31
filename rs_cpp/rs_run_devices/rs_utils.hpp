#ifndef RS_UTILS_HPP
#define RS_UTILS_HPP

// Based on : librealsense/examples/save-to-disk/rs-save-to-disk.cpp

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>  // File IO
#include <iostream> // Terminal IO
#include <sstream>  // Stringstreams

#include "utils.hpp"

/**
 * @brief Save raw frame data into a binary file.
 *
 * Taken from : https://github.com/IntelRealSense/librealsense/issues/1485
 *
 * @param frm An instance of rs2::frame .
 * @param filename name of file to save to.
 * @return true
 * @return false
 */
bool framedata_to_bin(const rs2::frame &frm, const std::string &filename);

/**
 * @brief Saves the metadata from rs2::frame into a csv file.
 *
 * @param frm An instance of rs2::frame .
 * @param filename name of file to save to.
 */
void metadata_to_csv(const rs2::frame &frm, const std::string &filename);

/**
 * @brief Creates a holder to collect important rs variables.
 *
 */
struct device
{
    std::shared_ptr<rs2::pipeline> pipeline;
    std::shared_ptr<rs2::pipeline_profile> pipeline_profile;
    std::shared_ptr<rs2::color_sensor> color_sensor;
    std::shared_ptr<rs2::depth_sensor> depth_sensor;
    rs2_metadata_type color_timestamp = 0;
    rs2_metadata_type depth_timestamp = 0;
    rs2_metadata_type color_reset_counter = 0;
    rs2_metadata_type depth_reset_counter = 0;
    int camera_temp_printout_counter = -1;
    int num_streams = 0;
};

/**
 * @brief Configs needed for rs stream.
 *
 */
struct stream_config
{
    rs2_stream stream_type = RS2_STREAM_COLOR; // RS2_STREAM_DEPTH;
    int width = 848;
    int height = 480;
    rs2_format format = RS2_FORMAT_BGR8; // RS2_FORMAT_YUYV; // RS2_FORMAT_Z16;
    int framerate = 30;
};

/**
 * @brief Storage paths to save data.
 *
 */
class storagepaths
{
public:
    time_t trial_idx;
    bool save = true;
    std::string timestamp;
    std::string calib;
    std::string color;
    std::string depth;
    std::string color_metadata;
    std::string depth_metadata;
    storagepaths();
    void create(const std::string &device_sn,
                const std::string &base_path);
    void show();
};

#endif
