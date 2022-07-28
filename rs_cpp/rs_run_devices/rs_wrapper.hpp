#ifndef RS_WRAPPER_HPP
#define RS_WRAPPER_HPP

// Based on : librealsense/examples/save-to-disk/rs-save-to-disk.cpp

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include <librealsense2/h/rs_pipeline.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>
#include <map>
#include <time.h>
#include <cstdlib>
#include <fstream>  // File IO
#include <iostream> // Terminal IO
#include <sstream>  // Stringstreams

#include "utils.hpp"
#include "rs_args.hpp"

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

/**
 * @brief Wrapper class for the librealsense library to run a realsense device.
 *
 */
class rs2wrapper : public rs2args
{
    // Device data
    std::shared_ptr<rs2::context> ctx;
    std::vector<std::string> available_devices_sn;
    std::vector<std::vector<std::string>> available_devices;
    std::vector<std::string> enabled_devices_sn;
    std::map<std::string, std::shared_ptr<device>> enabled_devices;
    // std::map calib_data;

    std::string single_device_sn = "-1";

    // RS align method
    // rs2::align align = align_to_color;

    // Configurations
    std::map<std::string, rs2::config> rs_cfg;
    stream_config stream_config_color;
    stream_config stream_config_depth;

    // Paths for saving data
    std::map<std::string, storagepaths> storagepaths_perdev;

    // Timestamp data
    int camera_temp_printout_interval = 3600;
    rs2_frame_metadata_value timestamp_mode = RS2_FRAME_METADATA_TIME_OF_ARRIVAL;
    std::chrono::steady_clock::time_point global_timestamp_start = std::chrono::steady_clock::now();

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // [INTERNAL] --------------------------------------------------------------
    // Reset frozen devices
    std::map<std::string, bool> reset_flags;
    // Output message
    std::vector<std::pair<std::string, std::string>> output_msg_list;
    // Frame check
    std::map<std::string, bool> valid_frame_check_flag;
    std::map<std::string, std::int64_t> empty_frame_check_counter;
    // -------------------------------------------------------------- [INTERNAL]

    /**
     * @brief Initialize the pipeline.
     *
     * @param ctx rs2::context object.
     */
    rs2::pipeline initialize_pipeline(const std::shared_ptr<rs2::context> context);

    /**
     * @brief configures the rs stream.
     *
     * @param device_sn device serial number.
     * @param stream_config_color stream config for color.
     * @param stream_config_depth stream config for depth.
     */
    void configure_stream(const std::string &device_sn,
                          const stream_config &stream_config_color,
                          const stream_config &stream_config_depth);

    /**
     * @brief processes the color and depth streams.
     *
     * @param device_sn device serial number.
     * @param frameset rs2 frameset object, contains multiple frames.
     * @param timestamp timestamp from rs.
     */
    void process_color_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              rs2_metadata_type &timestamp);
    void process_depth_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              rs2_metadata_type &timestamp);

    /**
     * @brief query the timestamp mode.
     *
     * @param device_sn device serial number.
     */
    void query_timestamp_mode(const std::string &device_sn);

    /**
     * @brief saves the timestamp.
     *
     * @param device_sn device serial number.
     * @param global_timestamp global system timestamp. (difference from start)
     * @param color_timestamp color timestamp from rs.
     * @param depth_timestamp depth timestamp from rs.
     */
    void save_timestamp(const std::string &device_sn,
                        const std::int64_t &global_timestamp,
                        const rs2_metadata_type &color_timestamp,
                        const rs2_metadata_type &depth_timestamp);

    /**
     * @brief print camera infos
     *
     * @param profile
     */
    void print_camera_infos(const std::shared_ptr<rs2::pipeline_profile> profile);
    void print_camera_temperature(const std::string &device_sn);

public:
    /**
     * @brief Construct a new rs2wrapper object
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     * @param ctx rs context.
     * @param device_sn Device serial number to be used.
     */
    rs2wrapper(int argc,
               char *argv[],
               rs2::context context,
               std::string device_sn = "-1");

    /**
     * @brief Initialize the realsense devices.
     *
     * @param enable_ir_emitter Whether to use the ir emmiter.
     * @param verbose Whether to printout infos.
     */
    void initialize(const bool &enable_ir_emitter = true,
                    const bool &verbose = true);
    void initialize(const std::string &device_sn,
                    const bool &enable_ir_emitter = true,
                    const bool &verbose = true);

    /**
     * @brief starts the rs pipeline.
     *
     */
    void start();
    void start(const std::string &device_sn);

    /**
     * @brief Collects a set of frames and postprocesses them.
     *
     * Currently this function polls for a set of frames and
     * saves the color & depth (as colormap) images as png,
     * and their metadata as csv.
     *
     */
    void step();
    void step(const std::string &device_sn);

    /**
     * @brief Stops the devices through rs2::pipeline
     *
     */
    void stop();
    void stop(const std::string &device_sn);

    /**
     * @brief resets the pipeline using stop and start.
     *
     */
    void reset();
    void reset(const std::string &device_sn);

    /**
     * @brief checks and resets if the reset counter gets too high.
     *
     */
    void reset_with_high_reset_counter();
    void reset_with_high_reset_counter(const std::string &device_sn);

    /**
     * @brief Saves the camera calibration data.
     *
     */
    void save_calib();
    void save_calib(const std::string &device_sn);

    /**
     * @brief Flushes N frames
     *
     * Used to discard frames at init to give autoexposure, etc. a chance to settle.
     *
     * @param num_frames N frames to flush.
     */
    void flush_frames(const int &num_frames = 30);
    void flush_frames(const std::string &device_sn,
                      const int &num_frames = 30);

    /**
     * @brief Get functions to expose member variables.
     *
     */
    std::string get_output_msg();
    std::vector<std::vector<std::string>> get_available_devices();
    std::vector<std::string> get_available_devices_sn();
    std::map<std::string, std::shared_ptr<device>> get_enabled_devices();
    std::vector<std::string> get_enabled_devices_sn();
};

#endif
