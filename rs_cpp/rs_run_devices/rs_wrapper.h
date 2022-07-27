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

#define NUM_ZEROS_TO_PAD 16;

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

struct stream_config
{
    rs2_stream stream_type = RS2_STREAM_COLOR; // RS2_STREAM_DEPTH;
    int width = 848;
    int height = 480;
    rs2_format format = RS2_FORMAT_BGR8; // RS2_FORMAT_YUYV; // RS2_FORMAT_Z16;
    int framerate = 30;
};

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
 * @brief Class that contains the arguments for the rs2wrapper class.
 *
 */
class rs2args : protected argparser
{

    std::map<std::string, rs2_format> _SUPPORTED_FORMATS{
        {"z16", RS2_FORMAT_Z16},
        {"bgr8", RS2_FORMAT_BGR8},
        {"rgb8", RS2_FORMAT_RGB8},
        {"yuyv", RS2_FORMAT_YUYV}};

public:
    /**
     * @brief Construct a new rs2args object
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     */
    rs2args(int argc, char **argv) : argparser(argc, argv)
    {
    }
    /**
     * @brief Steps to run the realsense device.
     *
     * @return int
     */
    int steps()
    {
        return std::stoi(getarg("--steps"));
    }
    /**
     * @brief FPS of the realsense device.
     *
     * @return int
     */
    int fps()
    {
        return std::stoi(getarg("--fps"));
    }
    /**
     * @brief Height of the realsense frames.
     *
     * @return int
     */
    int height()
    {
        return std::stoi(getarg("--height"));
    }
    /**
     * @brief Width of the realsense frames.
     *
     * @return int
     */
    int width()
    {
        return std::stoi(getarg("--width"));
    }
    rs2_format color_format()
    {
        if (_SUPPORTED_FORMATS.count(getarg("--color-format")))
            return _SUPPORTED_FORMATS[getarg("--color-format")];
        else
            throw std::invalid_argument("rs color format unknown");
    }
    rs2_format depth_format()
    {
        if (_SUPPORTED_FORMATS.count(getarg("--depth-format")))
            return _SUPPORTED_FORMATS[getarg("--depth-format")];
        else
            throw std::invalid_argument("rs depth format unknown");
    }
    /**
     * @brief Path to save the frames.
     *
     * @return std::string
     */
    std::string save_path()
    {
        return getarg("--save-path");
    }
    /**
     * @brief IP address of the remote realsense device (optional).
     *
     * @return const char*
     */
    const char *ip()
    {
        return getarg("--ip").c_str();
    }
    /**
     * @brief Checks whether the realsense device is connected over the network.
     *
     * @return true
     * @return false
     */
    bool network()
    {
        if (checkarg("--ip"))
            return true;
        else
            return false;
    }

    void print_args()
    {
        printout();
    }
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
     * @param device_sn
     * @param stream_config_color
     * @param stream_config_depth
     */
    void configure_stream(const std::string &device_sn,
                          const stream_config &stream_config_color,
                          const stream_config &stream_config_depth);
    void process_color_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              rs2_metadata_type &timestamp);
    void process_depth_stream(const std::string &device_sn,
                              const rs2::frameset &frameset,
                              rs2_metadata_type &timestamp);

    void query_timestamp_mode(const std::string &device_sn);
    void save_timestamp(const std::string &device_sn,
                        const std::int64_t &global_timestamp,
                        const rs2_metadata_type &color_timestamp,
                        const rs2_metadata_type &depth_timestamp);

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

    void reset();
    void reset(const std::string &device_sn);

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

    std::string get_output_msg();

    std::vector<std::vector<std::string>> get_available_devices();
    std::vector<std::string> get_available_devices_sn();
    std::map<std::string, std::shared_ptr<device>> get_enabled_devices();
    std::vector<std::string> get_enabled_devices_sn();
};