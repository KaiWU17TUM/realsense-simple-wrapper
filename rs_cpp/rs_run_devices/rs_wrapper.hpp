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

#include <mutex>
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

/**
 * @brief Wrapper class for the librealsense library to run a realsense device.
 *
 */
class rs2wrapper
{
    bool verbose = false;

    // Args from CLI
    rs2args args;

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

    // // Alignment of data from different streams.
    rs2::align align_to_color = rs2::align(RS2_STREAM_COLOR);
    rs2::align align_to_depth = rs2::align(RS2_STREAM_DEPTH);

    // [INTERNAL] --------------------------------------------------------------
    // Reset frozen devices
    std::map<std::string, bool> reset_flags;
    int max_reset_counter = 500;
    // Output message
    std::vector<std::pair<std::string, std::string>> output_msg_list;
    // Frame check
    std::map<std::string, bool> valid_frame_check_flag;
    std::map<std::string, int64_t> empty_frame_check_counter;
    // IP Mapping
    std::map<std::string, std::string> USBIP_MAPPING{
        {"001622070408", "192.168.1.238"},
        {"001622071039", "192.168.1.202"},
        {"001622070717", "192.168.1.34"}};
    // -------------------------------------------------------------- [INTERNAL]

    /**
     * @brief Base constructor function
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     * @param verbose If provided overrides the one in args.
     * @param ctx rs context.
     * @param device_sn Device serial number to be used.
     */
    void _constructor(int argc,
                      char *argv[],
                      const bool &verbose,
                      rs2::context context,
                      std::string device_sn);

    /**
     * @brief Initialize the pipeline.
     *
     */
    rs2::pipeline initialize_pipeline();

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
    bool process_color_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              rs2_metadata_type &timestamp);
    bool process_depth_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              rs2_metadata_type &timestamp);
    bool process_color_depth_stream(const std::string &device_sn,
                                    const rs2::frameset &frameset,
                                    rs2_metadata_type &color_timestamp,
                                    rs2_metadata_type &depth_timestamp);

    /**
     * @brief alsigns the frameset to either color or depth.
     *
     * @param device_sn device serial number.
     * @param frameset rs2 frameset object, contains multiple frames.
     * @param aligned_frameset rs2 frameset object that is aligned.
     * @param align_to rs2 align object, aligns frame.
     * @return bool whether the frameset has been successfully aligned.
     */
    bool align_frameset(const std::string &device_sn,
                        rs2::frameset &frameset,
                        rs2::frameset &aligned_frameset,
                        const int &mode = 0);

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
                        const int64_t &global_timestamp,
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
     * @param verbose If provided overrides the one in args.
     * @param ctx rs context.
     * @param device_sn Device serial number to be used.
     */
    rs2wrapper(int argc,
               char *argv[],
               const bool &verbose,
               rs2::context context,
               std::string device_sn = "-1");
    rs2wrapper(int argc,
               char *argv[],
               rs2::context context,
               std::string device_sn = "-1");

    /**
     * @brief
     *
     */
    void prepare_storage();

    /**
     * @brief prepares the config for the rs2 pipeline.
     *
     * @param device_sn device serial number.
     * @param stream_config_color stream config for color.
     * @param stream_config_depth stream config for depth.
     */
    void configure(const std::string &device_sn,
                   const stream_config &stream_config_color,
                   const stream_config &stream_config_depth);

    /**
     * @brief Initialize the realsense devices.
     *
     * @param enable_ir_emitter Whether to use the ir emmiter.
     */
    void initialize(const bool &enable_ir_emitter = true);
    void initialize(const std::string &device_sn,
                    const bool &enable_ir_emitter = true);
    void initialize_depth_sensor();
    void initialize_depth_sensor(const std::string &device_sn);

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
    void step_clear();
    bool step_receiving_frame_from_all_devices();

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
    void reset_hardware(const std::string &device_sn);

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
    void flush_frames(const int &num_frames = -1);
    void flush_frames(const std::string &device_sn,
                      const int &num_frames = -1);

    /**
     * @brief resets the global timestamp start
     *
     */
    void reset_global_timestamp();
    void reset_global_timestamp(std::chrono::steady_clock::time_point global_timestamp);

    /**
     * @brief Set functions to change member variables.
     *
     */
    void set_color_stream_config(const int &width, const int &height,
                                 const int &fps, const rs2_format &format);
    void set_depth_stream_config(const int &width, const int &height,
                                 const int &fps, const rs2_format &format);
    void set_valid_frame_check_flag(const std::string &device_sn,
                                    const bool &flag);
    void set_storagepaths_perdev(const std::map<std::string, storagepaths> &storagepaths_perdev);

    /**
     * @brief Get functions to expose member variables.
     *
     */
    int64_t get_empty_frame_check_counter(const std::string &device_sn);
    std::map<std::string, int64_t> get_empty_frame_check_counter();
    std::string get_output_msg();
    std::vector<std::vector<std::string>> get_available_devices();
    std::vector<std::string> get_available_devices_sn();
    std::map<std::string, std::shared_ptr<device>> get_enabled_devices();
    std::vector<std::string> get_enabled_devices_sn();
    rs2args get_args();
    rs2_metadata_type get_frame_timestamp(const std::string &device_sn,
                                          const rs2::frame &frame);
    std::map<std::string, storagepaths> get_storagepaths_perdev();

    /**
     * @brief Check functions to see if some condition is true.
     *
     * @return true
     * @return false
     */
    bool check_enabled_device(const std::string &device_sn,
                              const std::string &function_name);
    bool check_valid_color_depth_streams(const std::string &device_sn,
                                         const rs2::frameset &frameset);
};

#endif
