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
    int default_depth_sensor_autoexposure_limit = 200000;

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
        "--save-path",
    };

    std::map<std::string, std::string> _OPTIONAL_ARGS{
        {"--ip", ""},
        {"--flush-steps", "30"},
        {"--reset-interval", "120"},
        {"--verbose", "false"},
        {"--multithreading", "false"},
        {"--autoexposure", "true"},
        {"--depth-sensor-autoexposure-limit", std::to_string(default_depth_sensor_autoexposure_limit)},
        {"--enable-ir-emitter", "true"},
        {"--ir-emitter-power", "150"},
        {"--camera-temperature-printout-interval", "60"},
        {"--max-reset-counter", "500"},
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
        auto _arg = "--ip";
        return checkarg(_arg) ? getarg("--ip") : _OPTIONAL_ARGS[_arg];
    };

    /**
     * @brief Checks whether the realsense device is connected over the network.
     *
     * @return true
     * @return false
     */
    bool network()
    {
        auto _arg = "--ip";
        return checkarg(_arg) ? true : false;
    };

    /**
     * @brief steps to flush initial frames.
     *
     * @return int
     */
    int flush_steps()
    {
        auto _arg = "--flush-steps";
        return checkarg(_arg) ? getargi(_arg) : std::stoi(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief interval to reset pipeline.
     *
     * @return int
     */
    int reset_interval()
    {
        auto _arg = "--reset-interval";
        return checkarg(_arg) ? getargi(_arg) : std::stoi(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief printouts
     *
     * @return true
     * @return false
     */
    bool verbose()
    {
        auto _arg = "--verbose";
        return checkarg(_arg) ? getargb(_arg) : stob(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief multithreading mode. 1 thread per camera.
     *
     * @return true
     * @return false
     */
    bool multithreading()
    {
        auto _arg = "--multithreading";
        return checkarg(_arg) ? getargb(_arg) : stob(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief Autoexposure mode for depth and color sensors.
     *
     * @return true
     * @return false
     */
    bool autoexposure()
    {
        auto _arg = "--autoexposure";
        return checkarg(_arg) ? getargb(_arg) : stob(_OPTIONAL_ARGS[_arg]);
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
        return checkarg(_arg) ? getargi(_arg) : default_depth_sensor_autoexposure_limit;
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
        return checkarg(_arg) ? getargb(_arg) : stob(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief
     *
     * @return int
     */
    int ir_emitter_power()
    {
        auto _arg = "--ir-emitter-power";
        return checkarg(_arg) ? getargi(_arg) : std::stoi(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief Interval for printing camera temperature in sec.
     *
     * @return int
     */
    int camera_temperature_printout_interval()
    {
        auto _arg = "--camera-temperature-printout-interval";
        return checkarg(_arg) ? getargi(_arg) : std::stoi(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief Maximum reset counter expressed in millisec.
     *
     * @return int
     */
    int max_reset_counter()
    {
        auto _arg = "--max-reset-counter";
        return checkarg(_arg) ? getargi(_arg) : std::stoi(_OPTIONAL_ARGS[_arg]);
    };

    /**
     * @brief prints out the raw arguments.
     *
     */
    void print_args() { printout(); };

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
            if (!checkarg(arg.first))
                print(arg.first + " not defined, using default : " + arg.second, 1);

        std::cout << " " << std::endl;

        return EXIT_SUCCESS;
    };
};

#endif
