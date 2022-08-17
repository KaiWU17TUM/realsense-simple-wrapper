#ifndef RS_ARGS_HPP
#define RS_ARGS_HPP

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <map>
#include "utils.hpp"

/**
 * @brief Class that contains the arguments for the rs2wrapper class.
 *
 */
class rs2args : public argparser
{

public:
    std::map<std::string, rs2_format> _SUPPORTED_FORMATS{
        {"z16", RS2_FORMAT_Z16},
        {"bgr8", RS2_FORMAT_BGR8},
        {"rgb8", RS2_FORMAT_RGB8},
        {"yuyv", RS2_FORMAT_YUYV}};

    std::vector<std::string> _REQUIRED_ARGS{
        "--steps",
        "--fps",
        "--height",
        "--width",
        "--color-format",
        "--depth-format",
    };

    std::vector<std::string> _OPTIONAL_ARGS{
        "--reset-interval",
        "--flush-steps",
        "--ip",
        "--multithreading",
        "--verbose",
        "--save-path",
    };

    /**
     * @brief Construct a new rs2args object (empty)
     *
     */
    rs2args();
    /**
     * @brief Construct a new rs2args object
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     */
    rs2args(int argc, char *argv[]);
    /**
     * @brief Destroy the rs2args object
     *
     */
    ~rs2args();
    /**
     * @brief Steps to run the realsense device.
     *
     * @return int
     */
    int steps();
    /**
     * @brief FPS of the realsense device.
     *
     * @return int
     */
    int fps();
    /**
     * @brief Height of the realsense frames.
     *
     * @return int
     */
    int height();
    /**
     * @brief Width of the realsense frames.
     *
     * @return int
     */
    int width();
    /**
     * @brief checks and return color format for rs.
     *
     * @return rs2_format
     */
    rs2_format color_format();
    /**
     * @brief checks and return depth format for rs.
     *
     * @return rs2_format
     */
    rs2_format depth_format();
    /**
     * @brief Path to save the frames.
     *
     * @return std::string
     */
    std::string save_path();
    /**
     * @brief IP address of the remote realsense device (optional).
     *
     * @return std::string
     */
    std::string ip();
    /**
     * @brief Checks whether the realsense device is connected over the network.
     *
     * @return true
     * @return false
     */
    bool network();
    /**
     * @brief steps to flush initial frames.
     *
     * @return int
     */
    int flush_steps();
    /**
     * @brief interval to reset pipeline.
     *
     * @return int
     */
    int reset_interval();
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool verbose();
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool multithreading();
    /**
     * @brief prints out the raw arguments.
     *
     */
    void print_args();
    bool check_rs2args();
};

#endif
