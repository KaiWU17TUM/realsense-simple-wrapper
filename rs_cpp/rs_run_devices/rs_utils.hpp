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
#include <vector>
#include <map>
#include <cerrno>
#include <cstring>

#include "utils.hpp"

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
    std::string sn;
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
class storagepath
{
public:
    time_t trial_idx;
    bool save = true;
    std::vector<std::string> device_sns;
    std::map<std::string, std::string> timestamp;
    std::map<std::string, std::string> calib;
    std::map<std::string, std::string> color;
    std::map<std::string, std::string> depth;
    std::map<std::string, std::string> color_metadata;
    std::map<std::string, std::string> depth_metadata;
    storagepath();
    void create(const std::vector<std::string> &device_sns,
                const std::string &base_path);
    void show();
    void show(const std::string &device_sn);

private:
    int make_dirs(const char *path, const bool &exists_ok);
    void create(const std::string &device_sn,
                const std::string &base_path);
};

/**
 * @brief prints out rs camera infos.
 *
 * @param device
 * @param verbose
 */
void print_rs2_device_infos(const rs2::device &device, const bool &verbose);

/**
 * @brief prints out the temeprature of the camera asic and IR projector
 *
 * @param device
 * @param printout_interval
 * @param verbose
 */
void print_camera_temperature(device &device,
                              const int &printout_interval,
                              const bool &verbose);

/**
 * @brief Prints out a message saying no device enabled.
 *
 * @param msg message to be printed
 * @param mode which tags to use
 */
void print_no_device_enabled(const std::string &function);

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
 * @briefsaves timestamp to txt file.
 *
 * @param global_timestamp
 * @param color_timestamp
 * @param depth_timestamp
 * @param filename
 */
void timestamp_to_txt(const int64_t &global_timestamp,
                      const rs2_metadata_type &color_timestamp,
                      const rs2_metadata_type &depth_timestamp,
                      const std::string &filename);

/**
 * @brief check if color and depth frames are valid.
 *
 * @param frameset
 * @return true
 * @return false
 */
bool check_if_color_depth_frames_are_valid(const rs2::frameset &frameset);

#endif
