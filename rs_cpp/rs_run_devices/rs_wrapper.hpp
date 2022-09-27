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
#include "rs_utils.hpp"

/**
 * @brief Wrapper class for the librealsense library to run a realsense device.
 *
 */
class rs2wrapper
{

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
    rs2wrapper(rs2args args,
               const bool &verbose,
               rs2::context context,
               std::string device_sn = "-1");
    rs2wrapper(rs2args args,
               rs2::context context,
               std::string device_sn = "-1");

    /**
     * @brief Initialize the realsense devices.
     *
     * @param enable_ir_emitter Whether to use the ir emmiter.
     */
    void initialize(const bool &enable_ir_emitter = true,
                    const bool &set_roi = false);
    void initialize(const std::string &device_sn,
                    const bool &enable_ir_emitter = true,
                    const bool &set_roi = false);
    void initialize_depth_sensor_ae();
    void initialize_depth_sensor_ae(const std::string &device_sn);

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
    bool step_check_if_frames_are_valid(const std::string &device_sn,
                                        const rs2::frameset &frameset);

    /**
     * @brief Stops the devices through rs2::pipeline
     *
     */
    void stop();
    void stop(const std::string &device_sn);
    void stop_sensor();
    void stop_sensor(const std::string &device_sn);

    /**
     * @brief resets the pipeline using stop and start.
     *
     */
    void reset();
    void reset(const std::string &device_sn);
    void reset_hardware(const std::string &device_sn);
    void reset_reset_counter();
    void reset_reset_counter(const std::string &device_sn);

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
    void flush_frame();
    void flush_frame(const std::string &device_sn);
    void flush_frames(const int &num_frames = -1);

    /**
     * @brief resets the global timestamp start
     *
     */
    void reset_global_timestamp();
    void reset_global_timestamp(std::chrono::steady_clock::time_point global_timestamp);

    /**
     * @brief prepare the storage paths.
     *
     */
    void prepare_storage();

    /**
     * @brief Set functions to change member variables.
     *
     */
    void set_storagepaths(const std::map<std::string, storagepath> &storagepaths);

    /**
     * @brief Get functions to expose member variables.
     *
     */
    int64_t get_empty_frame_received_timers(const std::string &device_sn);
    std::map<std::string, int64_t> get_empty_frame_received_timers();
    std::string get_output_msg();
    std::vector<std::vector<std::string>> get_available_devices();
    std::map<std::string, std::shared_ptr<device>> get_enabled_devices();
    rs2args get_args();
    rs2_metadata_type get_frame_timestamp(const std::string &device_sn,
                                          const rs2::frame &frame);
    std::map<std::string, storagepath> get_storagepaths();

    /**
     * @brief Check functions to see if some condition is true.
     *
     * @return true
     * @return false
     */
    bool check_if_device_is_enabled(const std::string &device_sn,
                                    const std::string &function_name);

private:
    /**
     * @brief Base constructor function
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     * @param verbose If provided overrides the one in args.
     * @param ctx rs context.
     * @param device_sn Device serial number to be used.
     */
    void constructor(rs2args args,
                     const bool &verbose,
                     rs2::context context,
                     std::string device_sn);

    void query_available_devices();

    rs2::pipeline initialize_pipeline();

    /**
     * @brief configures the rs stream + sensor.
     *
     * @param device_sn device serial number.
     */
    void configure_color_stream_config(const std::string &device_sn);
    void configure_depth_stream_config(const std::string &device_sn);
    void configure_stream(const std::string &device_sn);
    void configure_color_sensor(const std::string &device_sn);
    void configure_depth_sensor(const std::string &device_sn);
    void configure_color_depth_sensor(const std::string &device_sn);
    void configure_ir_emitter(const std::string &device_sn);

    /**
     * @brief processes the color and depth streams.
     *
     * The individual processes returns false if
     * - there is any error,
     * - timestamp is not valid,
     * - timestamp is same as before (frozen).
     * This then causes 'process_color_depth_stream' to return
     * - 0 : no error
     * - 1 : color stream error
     * - 2 : depth stream error
     * - 3 : color and depth stream error.
     * Reset counter increment only in 'step(...)' .
     *
     * @param device_sn device serial number.
     * @param frameset rs2 frameset object, contains multiple frames.
     * @param global_timestamp timestamp from chrono.
     * @param timestamp timestamp from rs.
     */
    bool process_color_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              const int64_t &global_timestamp,
                              rs2_metadata_type &timestamp);
    bool process_depth_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              const int64_t &global_timestamp,
                              rs2_metadata_type &timestamp);
    int process_color_depth_stream(const std::string &device_sn,
                                   const rs2::frameset &frameset,
                                   const int64_t &global_timestamp,
                                   rs2_metadata_type &color_timestamp,
                                   rs2_metadata_type &depth_timestamp);

    /**
     * @brief aligns the frameset to either color or depth.
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

    // [MEMBER VARIABLES] ------------------------------------------------------
    bool verbose = false;

    // Args from CLI
    rs2args args;

    // Device data
    std::shared_ptr<rs2::context> ctx;
    std::vector<std::vector<std::string>> available_devices; // [serial + product line]
    std::map<std::string, std::shared_ptr<device>> enabled_devices;
    // std::map calib_data;

    std::string single_device_sn = "-1";

    // Configurations
    std::map<std::string, rs2::config> rs_cfg;
    std::map<std::string, stream_config> stream_config_colors;
    std::map<std::string, stream_config> stream_config_depths;

    // Paths for saving data
    std::map<std::string, storagepath> storagepaths;

    // Timestamp data
    int camera_temp_printout_interval = 3600;
    rs2_frame_metadata_value timestamp_mode = RS2_FRAME_METADATA_TIME_OF_ARRIVAL;
    std::chrono::steady_clock::time_point global_timestamp_start = std::chrono::steady_clock::now();

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Alignment of data from different streams.
    rs2::align align_to_color = rs2::align(RS2_STREAM_COLOR);
    rs2::align align_to_depth = rs2::align(RS2_STREAM_DEPTH);

    // [INTERNAL] --------------------------------------------------------------
    // Reset frozen devices
    int max_reset_counter = 500;
    // Output message
    std::vector<std::pair<std::string, std::string>> output_msgs;
    // Frame check, true if poll/wait returns a valid frame.
    std::map<std::string, bool> valid_frame_received_flags;
    std::map<std::string, int64_t> empty_frame_received_timers;
    // IP Mapping
    std::map<std::string, std::string> USBIP_MAPPING{
        {"001622070408", "192.168.1.238"},
        {"001622071039", "192.168.1.202"},
        {"001622070717", "192.168.1.34"}};
    // -------------------------------------------------------------- [INTERNAL]

    // ------------------------------------------------------ [MEMBER VARIABLES]
};

#endif
