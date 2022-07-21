// Based on : librealsense/examples/save-to-disk/rs-save-to-disk.cpp

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include <librealsense2/h/rs_pipeline.h>

#include <sys/types.h>
#include <sys/stat.h>

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
 * @brief pads a string with zeros.
 *
 * https://stackoverflow.com/questions/6143824/add-leading-zeros-to-string-without-sprintf
 *
 * @param in_str input string
 * @param num_zeros number of zeros to pad
 * @return std::string
 */
std::string pad_zeros(const std::string &in_str, const size_t &num_zeros);

/**
 * @brief Update prefix with timestamp from rs2 metadata.
 *
 * @param frm An instance of rs2::frame .
 * @param prefix Default prefix.
 * @param num_zeros number of zeros to pad
 * @return std::string
 */
std::string filename_prefix_with_timestamp(const rs2::frame &frm,
                                           const std::string &prefix,
                                           const size_t &num_zeros);

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
};

struct stream_config
{
    rs2_stream stream_type = RS2_STREAM_COLOR;
    // rs2_stream stream_type = RS2_STREAM_DEPTH;
    int width = 848;
    int height = 480;
    rs2_format format = RS2_FORMAT_BGR8;
    // rs2_format format = RS2_FORMAT_YUYV;
    // rs2_format format = RS2_FORMAT_Z16;
    int framerate = 30;
};

class storagepaths
{
public:
    bool save = true;
    std::string calib;
    std::string color;
    std::string depth;
    std::string color_metadata;
    std::string depth_metadata;
    storagepaths();

    /**
     * @brief Creates the required directories to save data
     *
     */
    void create_directories(const std::string &device_sn,
                            const std::string &base_path);
};

/**
 * @brief Class that contains the arguments for the rs2wrapper class.
 *
 */
class rs2args
{
    int _argc;
    char **_argv;

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
    rs2args(int argc, char *argv[])
    {
        _argc = argc;
        _argv = argv;
    }
    /**
     * @brief FPS of the realsense device.
     *
     * @return int
     */
    int fps()
    {
        return atoi(_argv[1]);
    }
    /**
     * @brief Height of the realsense frames.
     *
     * @return int
     */
    int height()
    {
        return atoi(_argv[2]);
    }
    /**
     * @brief Width of the realsense frames.
     *
     * @return int
     */
    int width()
    {
        return atoi(_argv[3]);
    }
    rs2_format color_format()
    {
        if (_SUPPORTED_FORMATS.count(std::string(_argv[4])))
            return _SUPPORTED_FORMATS[std::string(_argv[4])];
        else
            throw std::invalid_argument("rs color format unknown");
    }
    rs2_format depth_format()
    {
        if (_SUPPORTED_FORMATS.count(std::string(_argv[5])))
            return _SUPPORTED_FORMATS[std::string(_argv[5])];
        else
            throw std::invalid_argument("rs depth format unknown");
    }
    /**
     * @brief Path to save the frames.
     *
     * @return const char*
     */
    const char *save_path()
    {
        return _argv[6];
    }
    /**
     * @brief IP address of the remote realsense device (optional).
     *
     * @return const char*
     */
    const char *ip()
    {
        return _argv[7];
    }
    /**
     * @brief Checks whether the realsense device is connected over the network.
     *
     * @return true
     * @return false
     */
    bool network()
    {
        if (_argc == 8)
            return true;
        else
            return false;
    }
    /**
     * @brief Prints out the arguments from 'rs2args' class that is used for the 'rs2wrapper' class.
     *
     */
    void print_args()
    {
        std::cout << "========================================" << std::endl;
        std::cout << ">>>>> rs2args <<<<<" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "FPS        : " << fps() << std::endl;
        std::cout << "Height     : " << height() << std::endl;
        std::cout << "Width      : " << width() << std::endl;
        std::cout << "Save Path  : " << save_path() << std::endl;
        if (network())
            std::cout << "IP Address : " << ip() << std::endl;
        else
            std::cout << "IP Address : " << std::endl;
        std::cout << "========================================" << std::endl;
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
    std::vector<std::vector<std::string>> available_devices;
    std::map<std::string, device> enabled_devices;
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

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

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
     */
    void initialize(bool enable_ir_emitter = true, bool verbose = true);
    /**
     * @brief Initialize the pipeline.
     *
     * @param ctx
     * @return rs2::pipeline
     */
    rs2::pipeline initialize_pipeline(const std::shared_ptr<rs2::context> context);
    /**
     * @brief Flushes N initial frames o give autoexposure, etc. a chance to settle.
     *
     */
    void initial_flush(const int &num_frames = 30);
    /**
     * @brief Collects a set of frames and postprocesses them.
     *
     * Currently this function polls for a set of frames and
     * saves the color & depth (as colormap) images as png,
     * and their metadata as csv.
     *
     * @param save_file_prefix Prefix for the save file.
     */
    void step(const std::string &save_file_prefix);

    void save_calib();

    void configure_stream(const std::string &device_sn,
                          const stream_config &stream_config_color,
                          const stream_config &stream_config_depth);

    void print_camera_infos(const std::shared_ptr<rs2::pipeline_profile> profile);
};
