// Based on : librealsense/examples/save-to-disk/rs-save-to-disk.cpp

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include <librealsense2/h/rs_pipeline.h>

#include <sys/types.h>
#include <sys/stat.h>

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
 * @brief Class that contains the arguments for the rs2wrapper class.
 *
 */
class rs2args
{
    int _argc;
    char **_argv;

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
    /**
     * @brief Path to save the frames.
     *
     * @return const char*
     */
    const char *save_path()
    {
        return _argv[4];
    }
    /**
     * @brief IP address of the remote realsense device (optional).
     *
     * @return std::string
     */
    std::string ip()
    {
        return (std::string)_argv[5];
    }
    /**
     * @brief Checks whether the realsense device is connected over the network.
     *
     * @return true
     * @return false
     */
    bool network()
    {
        if (_argc == 6)
            return true;
        else
            return false;
    }
    /**
     * @brief Prints out the arguments from 'rs2args' class that is used for the 'rs2wrapper' class.
     *
     */
    void info()
    {
        std::cout << "========================================" << std::endl;
        std::cout << ">>>>> rs2args <<<<<" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "FPS        : " << fps() << std::endl;
        std::cout << "Height     : " << height() << std::endl;
        std::cout << "Width      : " << width() << std::endl;
        std::cout << "Save Path  : " << save_path() << std::endl;
        std::cout << "IP Address : " << ip() << std::endl;
        std::cout << "========================================" << std::endl;
    }
};

/**
 * @brief Wrapper class for the librealsense library to run a realsense device.
 *
 */
class rs2wrapper : public rs2args
{

    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Paths for saving data
    std::map<std::string, std::string> path_map{
        {"Color", ""},
        {"Depth", ""},
        {"MetaColor", ""},
        {"MetaDepth", ""},
        {"Calib", ""}};

public:
    /**
     * @brief Construct a new rs2wrapper object
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     * @param ctx An insstance of rs2::context .
     */
    rs2wrapper(int argc, char *argv[], rs2::context ctx = rs2::context());
    /**
     * @brief Creates the required directories to save data
     *
     */
    void create_directories();
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
};
