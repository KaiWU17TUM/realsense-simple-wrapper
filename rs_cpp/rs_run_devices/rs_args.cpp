// #include "rs_args.hpp"

// rs2args::rs2args() : argparser()
// {
// }

// rs2args::rs2args(int argc, char *argv[]) : argparser(argc, argv)
// {
// }

// rs2args::~rs2args()
// {
// }

// // [REQUIRED] ------------------------------------------------------------------

// int rs2args::steps()
// {
//     return getargi("--steps");
// }

// int rs2args::fps()
// {
//     return getargi("--fps");
// }

// int rs2args::height()
// {
//     return getargi("--height");
// }

// int rs2args::width()
// {
//     return getargi("--width");
// }

// rs2_format rs2args::color_format()
// {
//     auto f = getarg("--color-format");
//     if (_SUPPORTED_FORMATS.find(f) != _SUPPORTED_FORMATS.end())
//         return _SUPPORTED_FORMATS[f];
//     else
//         throw std::invalid_argument("rs color format unknown");
// }

// rs2_format rs2args::depth_format()
// {
//     auto f = getarg("--depth-format");
//     if (_SUPPORTED_FORMATS.find(f) != _SUPPORTED_FORMATS.end())
//         return _SUPPORTED_FORMATS[f];
//     else
//         throw std::invalid_argument("rs depth format unknown");
// }

// // ------------------------------------------------------------------ [REQUIRED]

// // [OPTIONAL] ------------------------------------------------------------------

// // int rs2args::reset_interval()
// // {
// //     auto _arg = "--reset-interval";
// //     return checkarg(_arg) ? getargi(_arg) : 120;
// // }

// // int rs2args::flush_steps()
// // {
// //     auto _arg = "--flush-steps";
// //     return checkarg(_arg) ? getargi(_arg) : 30;
// // }

// // std::string rs2args::ip()
// // {
// //     return getarg("--ip");
// // }

// // bool rs2args::network()
// // {
// //     return checkarg("--ip") ? true : false;
// // }

// // bool rs2args::multithreading()
// // {
// //     auto _arg = "--multithreading";
// //     return checkarg(_arg) ? getargb(_arg) : false;
// // }

// // bool rs2args::verbose()
// // {
// //     auto _arg = "--verbose";
// //     return checkarg(_arg) ? getargb(_arg) : false;
// // }

// // std::string rs2args::save_path()
// // {
// //     return getarg("--save-path");
// // }

// // bool rs2args::autoexposure()
// // {
// //     auto _arg = "--autoexposure";
// //     return checkarg(_arg) ? getargb(_arg) : true;
// // }

// // int rs2args::depth_sensor_autoexposure_limit()
// // {
// //     // Defaults to full range
// //     // https://github.com/IntelRealSense/librealsense/issues/10771
// //     auto _arg = "--depth-sensor-autoexposure-limit";
// //     return checkarg(_arg) ? getargi(_arg) : 200000;
// // }

// // bool rs2args::enable_ir_emitter()
// // {
// //     auto _arg = "--enable-ir-emitter";
// //     return checkarg(_arg) ? getargb(_arg) : true;
// // }

// // ------------------------------------------------------------------ [OPTIONAL]

// // void rs2args::print_args()
// // {
// //     printout();
// // }

// // bool rs2args::check_rs2args()
// // {
// //     for (auto &&arg : _REQUIRED_ARGS)
// //     {
// //         if (!checkarg(arg))
// //         {
// //             print(arg + " is missing", 2);
// //             return EXIT_FAILURE;
// //         }
// //     }
// //     for (auto &&arg : _OPTIONAL_ARGS)
// //         if (!checkarg(arg))
// //             print(arg + " is not used", 1);
// //     return EXIT_SUCCESS;
// // }