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
        "--autoexposure",
        "--depth-sensor-autoexposure-limit",
        "--enable-ir-emitter",
        "--ir-emitter-power",
    };

    /**
     * @brief Construct a new rs2args object (empty)
     *
     */
    rs2args(){};
    /**
     * @brief Construct a new rs2args object
     *
     * @param argc Number of arguments.
     * @param argv Arguments in an array of char*.
     */
    rs2args(int argc, char *argv[]) : argparser(argc, argv){};
    /**
     * @brief Destroy the rs2args object
     *
     */
    ~rs2args(){};
    /**
     * @brief Steps to run the realsense device.
     *
     * @return int
     */
    int steps()
    {
        return getargi("--steps");
    };
    /**
     * @brief FPS of the realsense device.
     *
     * @return int
     */
    int fps()
    {
        return getargi("--fps");
    };
    /**
     * @brief Height of the realsense frames.
     *
     * @return int
     */
    int height()
    {
        return getargi("--height");
    };
    /**
     * @brief Width of the realsense frames.
     *
     * @return int
     */
    int width()
    {
        return getargi("--width");
    };
    /**
     * @brief checks and return color format for rs.
     *
     * @return rs2_format
     */
    rs2_format color_format()
    {
        auto f = getarg("--color-format");
        if (_SUPPORTED_FORMATS.find(f) != _SUPPORTED_FORMATS.end())
            return _SUPPORTED_FORMATS[f];
        else
            throw std::invalid_argument("rs color format unknown");
    };
    /**
     * @brief checks and return depth format for rs.
     *
     * @return rs2_format
     */
    rs2_format depth_format()
    {
        auto f = getarg("--depth-format");
        if (_SUPPORTED_FORMATS.find(f) != _SUPPORTED_FORMATS.end())
            return _SUPPORTED_FORMATS[f];
        else
            throw std::invalid_argument("rs depth format unknown");
    };
    /**
     * @brief Path to save the frames.
     *
     * @return std::string
     */
    std::string save_path()
    {
        return getarg("--save-path");
    };
    /**
     * @brief IP address of the remote realsense device (optional).
     *
     * @return std::string
     */
    std::string ip()
    {
        return getarg("--ip");
    };
    /**
     * @brief Checks whether the realsense device is connected over the network.
     *
     * @return true
     * @return false
     */
    bool network()
    {
        return checkarg("--ip") ? true : false;
    };
    /**
     * @brief steps to flush initial frames.
     *
     * @return int
     */
    int flush_steps()
    {
        auto _arg = "--flush-steps";
        return checkarg(_arg) ? getargi(_arg) : 30;
    };
    /**
     * @brief interval to reset pipeline.
     *
     * @return int
     */
    int reset_interval()
    {
        auto _arg = "--reset-interval";
        return checkarg(_arg) ? getargi(_arg) : 120;
    };
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool verbose()
    {
        auto _arg = "--verbose";
        return checkarg(_arg) ? getargb(_arg) : false;
    };
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool multithreading()
    {
        auto _arg = "--multithreading";
        return checkarg(_arg) ? getargb(_arg) : false;
    };
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool autoexposure()
    {
        auto _arg = "--autoexposure";
        return checkarg(_arg) ? getargb(_arg) : true;
    };
    /**
     * @brief
     *
     * @return int
     */
    int depth_sensor_autoexposure_limit()
    {
        // Defaults to full range
        // https://github.com/IntelRealSense/librealsense/issues/10771
        auto _arg = "--depth-sensor-autoexposure-limit";
        return checkarg(_arg) ? getargi(_arg) : 200000;
    };
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool enable_ir_emitter()
    {
        auto _arg = "--enable-ir-emitter";
        return checkarg(_arg) ? getargb(_arg) : true;
    };
    /**
     * @brief
     *
     * @return true
     * @return false
     */
    int ir_emitter_power()
    {
        auto _arg = "--ir-emitter-power";
        return checkarg(_arg) ? getargi(_arg) : 150;
    };

    /**
     * @brief prints out the raw arguments.
     *
     */
    void print_args()
    {
        printout();
    };

    /**
     * @brief check if the args are valid.
     *
     * @return true
     * @return false
     */
    bool check_rs2args()
    {
        for (auto &&arg : _REQUIRED_ARGS)
        {
            if (!checkarg(arg))
            {
                print(arg + " is missing", 2);
                return EXIT_FAILURE;
            }
        }
        for (auto &&arg : _OPTIONAL_ARGS)
            if (!checkarg(arg))
                print(arg + " is not used", 1);
        return EXIT_SUCCESS;
    };
};

#endif
