#include "rs_wrapper.hpp"

std::mutex reset_mux;

/*******************************************************************************
 * rs2wrapper PUBLIC FUNCTIONS
 ******************************************************************************/
rs2wrapper::rs2wrapper(int argc,
                       char *argv[],
                       rs2::context context,
                       std::string device_sn)
{
    rs2args _args = rs2args(argc, argv);
    constructor(_args, _args.verbose(), context, device_sn);
}

rs2wrapper::rs2wrapper(int argc,
                       char *argv[],
                       const bool &verbose,
                       rs2::context context,
                       std::string device_sn)
{
    rs2args _args = rs2args(argc, argv);
    constructor(_args, verbose, context, device_sn);
}

rs2wrapper::rs2wrapper(rs2args args,
                       rs2::context context,
                       std::string device_sn)
{
    constructor(args, args.verbose(), context, device_sn);
}

rs2wrapper::rs2wrapper(rs2args args,
                       const bool &verbose,
                       rs2::context context,
                       std::string device_sn)
{
    constructor(args, verbose, context, device_sn);
}

void rs2wrapper::initialize(const bool &enable_ir_emitter,
                            const bool &set_roi)
{
    for (auto const &available_device : available_devices)
        initialize(available_device[0], enable_ir_emitter, set_roi);
}

void rs2wrapper::initialize(const std::string &device_sn,
                            const bool &enable_ir_emitter,
                            const bool &set_roi)
{
    // reset after 0.5s
    max_reset_counter = std::round(0.5 * (float)args.fps());

    print("Initializing RealSense devices " + std::string(device_sn), 0);

    // 0. enabled devices
    std::shared_ptr<device> dev = std::make_shared<device>();
    enabled_devices[device_sn] = dev;

    // 1. pipeline
    rs2::pipeline pipe = initialize_pipeline();
    dev->pipeline = std::make_shared<rs2::pipeline>(pipe);

    // 2. configure stream
    configure_color_stream_config(device_sn);
    configure_depth_stream_config(device_sn);
    configure_stream(device_sn);

    // 3. pipeline start
    start(device_sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (verbose)
        print("pipeline started with 100ms sleep...", 0);

    // 4. sensors
    configure_color_depth_sensor(device_sn);

    // 5. IR
    configure_ir_emitter(device_sn);

    // 6. align instances recreate
    align_to_color = rs2::align(RS2_STREAM_COLOR);
    align_to_depth = rs2::align(RS2_STREAM_DEPTH);

    // 7. infos
    print_camera_infos(dev->pipeline_profile);
    print_camera_temperature(device_sn);

    // 8. timestamp mode
    if (storagepaths.size() > 0)
        query_timestamp_mode(std::string(device_sn));

    print("Initialized RealSense devices " + std::string(device_sn) + "\n", 0);
}

void rs2wrapper::initialize_depth_sensor_ae()
{
    for (auto const &available_device : available_devices)
        initialize_depth_sensor_ae(available_device[0]);
}

void rs2wrapper::initialize_depth_sensor_ae(const std::string &device_sn)
{
    print("Initializing RealSense depth sensor AE " + std::string(device_sn), 0);

    // 0. enabled devices
    std::shared_ptr<device> dev = std::make_shared<device>();
    enabled_devices[device_sn] = dev;

    // 1. pipeline
    rs2::pipeline pipe = initialize_pipeline();
    dev->pipeline = std::make_shared<rs2::pipeline>(pipe);

    // 2. configure
    configure_color_stream_config(device_sn);
    configure_depth_stream_config(device_sn);
    // HACK: sensor AE configuration seems to only work for fps = 6
    stream_config_colors[device_sn].framerate = 6;
    stream_config_depths[device_sn].framerate = 6;
    configure_stream(device_sn);

    // 3. pipeline start
    start(device_sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (verbose)
        print("pipeline started with 100ms sleep...", 0);

    // 4. sensors
    std::vector<rs2::sensor> sensors =
        dev->pipeline_profile->get_device().query_sensors();
    for (auto &&sensor : sensors)
    {
        if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
        {
            // int limit = 8500; // default
            int limit = args.depth_sensor_autoexposure_limit();
            dss.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
            dss.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE, 1.0f);
            dss.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, float(limit));
            print("depth sensor auto exposure limit : " + std::to_string(limit), 0);
        }
    }

    // 5. pipeline stop
    stop(device_sn);

    print("Initialized RealSense depth sensor AE " + std::string(device_sn) + "\n", 0);
}

void rs2wrapper::start()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            start(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::start(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];
    rs2::pipeline_profile profile = dev->pipeline->start(rs_cfg[device_sn]);
    if (verbose)
        print(device_sn + " started...", 0);

    dev->pipeline_profile = std::make_shared<rs2::pipeline_profile>(profile);
    dev->num_streams = dev->pipeline_profile->get_streams().size();
    if (verbose)
        print(device_sn + " pipeline profile is saved...", 0);
}

void rs2wrapper::step()
{
    step_clear();

    while (valid_frame_received_flags.size() < enabled_devices.size())
    {
        for (auto const &enabled_device : enabled_devices)
        {
            auto device_sn = enabled_device.first;
            step(device_sn);
            reset_with_high_reset_counter(device_sn);
            // In case a device is not sending anything at all.
            // empty frame for 0.5 seconds.
            if (empty_frame_received_timers[device_sn] > 5e8)
            {
                empty_frame_received_timers[device_sn] = 0;
                reset(device_sn);
            }
        }
    }
}

void rs2wrapper::step(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::string output_msg = "";
    rs2::frameset frameset;
    rs2::frameset aligned_frameset;
    rs2_metadata_type current_color_timestamp = 0;
    rs2_metadata_type current_depth_timestamp = 0;

    // Getting the enabled device 'device' class object
    std::shared_ptr<device> dev = enabled_devices[device_sn];

    // Timestamp used to track empty frames.
    std::chrono::steady_clock::time_point local_timestamp =
        std::chrono::steady_clock::now();

    // Poll for frames.
    bool valid_frame = dev->pipeline->poll_for_frames(&frameset);

    // Polled frames are empty.
    if (!valid_frame)
    {
        empty_frame_received_timers[device_sn] +=
            get_timestamp_duration_ns(local_timestamp);
        // output_msg = device_sn + " :: Invalid frame...";
    }
    // Polled frames are not empty.
    else
    {
        // Make sure both color and depth frames are there.
        if (frameset.size() == dev->num_streams)
        {
            // Timestamp duration using the global start timestamp.
            // int64_t global_timestamp_diff = get_timestamp_duration_ns(
            //     global_timestamp_start);
            int64_t global_timestamp_diff = get_timestamp_ns();

            // Check if both frames are valid, skip step if one is invalid.
            if (!step_check_if_frames_are_valid(device_sn, frameset))
            {
                dev->color_reset_counter += 1;
                dev->depth_reset_counter += 1;
                output_msg = device_sn + " :: One of the streams is missing...";
            }
            // Both color and depth streams are valid.
            else
            {
                // Tries to align the frames, and skip step if exception occurs.
                if (!align_frameset(device_sn, frameset, aligned_frameset, 1))
                {
                    dev->color_reset_counter += 1;
                    dev->depth_reset_counter += 1;
                    output_msg = device_sn + " :: Align failed...";
                }
                // Framesets are aligned.
                else
                {
                    // Loops through the streams to get color and depth.
                    int error_status = process_color_depth_stream(
                        device_sn,
                        aligned_frameset,
                        global_timestamp_diff,
                        current_color_timestamp,
                        current_depth_timestamp);

                    // Something was wrong with color stream.
                    if (error_status == 1 || error_status == 3)
                    {
                        dev->color_reset_counter += 1;
                        if (verbose)
                            print(
                                device_sn +
                                    " Resetting, error in processing color stream, c=" +
                                    std::to_string(dev->color_reset_counter),
                                1);
                        output_msg = device_sn + " :: Error in stream...";
                    }

                    // Something was wrong with depth stream.
                    if (error_status == 2 || error_status == 3)
                    {
                        dev->depth_reset_counter += 1;
                        if (verbose)
                            print(
                                device_sn +
                                    " Resetting, error in processing depth stream, c=" +
                                    std::to_string(dev->depth_reset_counter),
                                1);
                        output_msg = device_sn + " :: Error in stream...";
                    }

                    // nothing is wrong with stream
                    // Saves the timestamps and generate output message.
                    if (error_status == 0)
                    {
                        save_timestamp(device_sn,
                                       global_timestamp_diff,
                                       current_color_timestamp,
                                       current_depth_timestamp);

                        valid_frame_received_flags[device_sn] = true;
                        empty_frame_received_timers[device_sn] = 0;

                        output_msg =
                            device_sn + "::" +
                            std::to_string(global_timestamp_diff) + "::" +
                            std::to_string(current_color_timestamp) + "::" +
                            std::to_string(current_depth_timestamp) + "  ";
                    }
                }
            }
        }
        // If we include this it will spam the terminal.
        // else
        // {
        //     output_msg = device_sn + " :: frameset missing color/depth frame...";
        // }
    }

    // Message to be printed out.
    std::pair<std::string, std::string> msg(device_sn, output_msg);
    output_msgs.push_back(msg);
}

void rs2wrapper::step_clear()
{
    // internal variables
    output_msgs.clear();
    valid_frame_received_flags.clear();
    empty_frame_received_timers.clear();
}

bool rs2wrapper::step_check_if_frames_are_valid(const std::string &device_sn,
                                                const rs2::frameset &frameset)
{
    rs2::frame cf = frameset.first_or_default(RS2_STREAM_COLOR);
    rs2::frame df = frameset.first_or_default(RS2_STREAM_DEPTH);
    if (!cf || !df)
        return false;
    else
        return true;
}

void rs2wrapper::stop()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            stop(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::stop(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    enabled_devices[device_sn]->pipeline->stop();
    if (verbose)
        print(device_sn + " stopped...", 0);
}

void rs2wrapper::stop_sensor()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            stop_sensor(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::stop_sensor(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    enabled_devices[device_sn]->color_sensor->stop();
    enabled_devices[device_sn]->depth_sensor->stop();
    if (verbose)
        print(device_sn + " stopped...", 0);
}

void rs2wrapper::reset()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            reset(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::reset(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    stop(device_sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (verbose)
        print(device_sn + " pipeline stopped + paused with 500ms sleep...", 0);

    start(device_sn);
    if (verbose)
        print(device_sn + " pipeline restarted...", 0);
}

void rs2wrapper::reset_hardware(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::lock_guard<std::mutex> lock(reset_mux);

    stop(device_sn);
    rs2::device dev = enabled_devices[device_sn]->pipeline_profile->get_device();

    dev.hardware_reset();
    if (verbose)
        print(device_sn + " hardware reset...", 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::string cmd = "~/realsense-simple-wrapper/scripts/pi4_client_bind.sh " + USBIP_MAPPING[device_sn];
    system(cmd.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    rs2::pipeline pipe = initialize_pipeline();
    enabled_devices[device_sn]->pipeline = std::make_shared<rs2::pipeline>(pipe);
    if (verbose)
        print(device_sn + " pipeline reinitialized...", 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    start(device_sn);
    if (verbose)
        print(device_sn + " pipeline restarted with 300ms sleep...", 0);
}

void rs2wrapper::reset_reset_counter()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            reset_reset_counter(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::reset_reset_counter(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    enabled_devices[device_sn]->color_reset_counter = 0;
    enabled_devices[device_sn]->depth_reset_counter = 0;
}

void rs2wrapper::reset_with_high_reset_counter()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            reset_with_high_reset_counter(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::reset_with_high_reset_counter(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    if (enabled_devices[device_sn]->color_reset_counter >= max_reset_counter)
    {
        if (verbose)
            print("Reset " + device_sn + " due to high color stream reset counter", 1);
        reset(device_sn);
        reset_reset_counter(device_sn);
    }
    else if (enabled_devices[device_sn]->depth_reset_counter >= max_reset_counter)
    {
        if (verbose)
            print("Reset " + device_sn + " due to high depth stream reset counter", 1);
        reset(device_sn);
        reset_reset_counter(device_sn);
    }
}

void rs2wrapper::save_calib()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            save_calib(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::save_calib(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];
    std::string csv_file = storagepaths[device_sn].calib + "/calib.csv";
    std::ofstream csv;
    csv.open(csv_file);

    // Intrinsics of color & depth frames
    rs2::stream_profile profile_color =
        dev->pipeline_profile->get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics intr_color =
        profile_color.as<rs2::video_stream_profile>().get_intrinsics();
    // Fetch stream profile for depth stream
    // Downcast to video_stream_profile and fetch intrinsics
    rs2::stream_profile profile_depth =
        dev->pipeline_profile->get_stream(RS2_STREAM_DEPTH);
    rs2_intrinsics intr_depth =
        profile_depth.as<rs2::video_stream_profile>().get_intrinsics();

    // Extrinsic matrix from color sensor to Depth sensor
    rs2_extrinsics extr =
        profile_color.as<rs2::video_stream_profile>()
            .get_extrinsics_to(profile_depth);

    // Write calibration data to json file
    csv << intr_color.width << ","
        << intr_color.height << ","
        << intr_color.ppx << ","
        << intr_color.ppy << ","
        << intr_color.fx << ","
        << intr_color.fy << ","
        << rs2_distortion_to_string(intr_color.model) << ",";
    for (auto const &value : intr_color.coeffs)
        csv << value << ",";
    csv << args.color_format() << ","
        << args.fps() << ",";
    csv << "\n";

    csv << intr_depth.width << ","
        << intr_depth.height << ","
        << intr_depth.ppx << ","
        << intr_depth.ppy << ","
        << intr_depth.fx << ","
        << intr_depth.fy << ","
        << rs2_distortion_to_string(intr_depth.model) << ",";
    for (auto const &value : intr_depth.coeffs)
        csv << value << ",";
    csv << args.depth_format() << ","
        << args.fps() << ",";
    csv << "\n";

    for (auto const &value : extr.rotation)
        csv << value << ",";
    for (auto const &value : extr.translation)
        csv << value << ",";
    csv << "\n";

    std::vector<rs2::sensor> sensors =
        dev->pipeline_profile->get_device().query_sensors();
    for (auto &&sensor : sensors)
    {
        if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
        {
            csv << dss.get_depth_scale() << ","
                << dss.get_stereo_baseline() << ",\n";
        }
    }

    print(device_sn + " Saved camera calibration data...", 0);
}

void rs2wrapper::flush_frames(const int &num_frames)
{
    int _num_frames = 0;
    if (num_frames == -1)
        _num_frames = args.flush_steps();
    else
        _num_frames = num_frames;

    for (auto i = 0; i < _num_frames; ++i)
        flush_frame();

    if (verbose)
        print("Flushed " + std::to_string(_num_frames) + " initial frames...", 0);
}

void rs2wrapper::flush_frame()
{
    if (enabled_devices.size() > 0)
        for (auto const &enabled_device : enabled_devices)
            flush_frame(enabled_device.first);
    else
        print_no_device_enabled(__func__);
}

void rs2wrapper::flush_frame(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];
    dev->pipeline->wait_for_frames();
}

void rs2wrapper::reset_global_timestamp()
{
    this->global_timestamp_start = std::chrono::steady_clock::now();
}

void rs2wrapper::reset_global_timestamp(std::chrono::steady_clock::time_point global_timestamp)
{
    this->global_timestamp_start = global_timestamp;
}

void rs2wrapper::prepare_storage()
{
    // storage
    for (auto const &available_device : available_devices)
    {
        auto device_sn = available_device[0];
        storagepath sp;
        sp.create(device_sn, args.save_path());
        storagepaths[device_sn] = sp;
    }
}

/*******************************************************************************
 * rs2wrapper PUBLIC FUNCTIONS : SET, GET, CHECK
 ******************************************************************************/

void rs2wrapper::set_storagepaths(const std::map<std::string, storagepath> &storagepaths)
{
    this->storagepaths = storagepaths;
}

int64_t rs2wrapper::get_empty_frame_received_timers(const std::string &device_sn)
{
    return this->empty_frame_received_timers[device_sn];
}

std::map<std::string, int64_t> rs2wrapper::get_empty_frame_received_timers()
{
    return this->empty_frame_received_timers;
}

std::string rs2wrapper::get_output_msg()
{
    std::string output_msg;
    std::sort(this->output_msgs.begin(), this->output_msgs.end());
    for (auto const &_output_msg : this->output_msgs)
        output_msg += _output_msg.second;
    return output_msg;
}

std::vector<std::vector<std::string>> rs2wrapper::get_available_devices()
{
    return this->available_devices;
}

std::map<std::string, std::shared_ptr<device>> rs2wrapper::get_enabled_devices()
{
    return this->enabled_devices;
}

rs2args rs2wrapper::get_args()
{
    return this->args;
}

rs2_metadata_type rs2wrapper::get_frame_timestamp(const std::string &device_sn,
                                                  const rs2::frame &frame)
{
    try
    {
        return frame.get_frame_metadata(this->timestamp_mode);
    }
    catch (const rs2::error &e)
    {
        std::string _msg = device_sn + " :: " +
                           e.get_failed_function() +
                           "(" + e.get_failed_args() + "): " +
                           e.what();
        print(_msg, 2);
        return -1;
    }
    catch (const std::exception &e)
    {
        print(e.what(), 2);
        return -1;
    }
}

std::map<std::string, storagepath> rs2wrapper::get_storagepaths()
{
    return this->storagepaths;
}

bool rs2wrapper::check_if_device_is_enabled(const std::string &device_sn,
                                            const std::string &function_name)
{
    if (enabled_devices.count(device_sn) <= 0)
    {
        print(device_sn + " is not enabled, skipping '" + function_name + "' ...", 1);
        return false;
    }
    else
        return true;
}

/*******************************************************************************
 * rs2wrapper PRIVATAE FUNCTIONS
 ******************************************************************************/
void rs2wrapper::constructor(rs2args args,
                             const bool &verbose,
                             rs2::context context,
                             std::string device_sn)
{
    // CLI args
    this->args = args;

    // whether to printout stuffs
    this->verbose = verbose;

    // if arg is given, we use only one rs device
    this->single_device_sn = device_sn;

    // Get available devices
    // context grabs the usb resources of the cameras.
    this->ctx = std::make_shared<rs2::context>(context);
    this->query_available_devices();

    // Sort the devices.
    std::sort(this->available_devices.begin(),
              this->available_devices.end(),
              [](const std::vector<std::string> &a,
                 const std::vector<std::string> &b)
              {
                  return a[0] < b[0];
              });
}

void rs2wrapper::query_available_devices()
{
    auto add_device =
        [](std::string serial,
           std::string product_line,
           std::vector<std::vector<std::string>> &available_devices,
           bool verbose)
    {
        std::vector<std::string> available_device{serial, product_line};
        available_devices.push_back(available_device);
        if (verbose)
            print("using : " + std::string(serial), 0);
    };

    if (args.network())
    {
        print("Network mode", 0);
        rs2::net_device dev(args.ip());
        if (verbose)
            print("Network device found", 0);
        dev.add_to(*ctx);

        auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        auto product_line = args.ip();
        add_device(serial, product_line, available_devices, verbose);
    }
    else
    {
        print("Local mode", 0);

        if (single_device_sn == "-1")
        {
            for (auto &&dev : ctx->query_devices())
            {
                auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                auto product_line = dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
                add_device(serial, product_line, available_devices, verbose);
            }
        }
        else
        {
            auto serial = single_device_sn;
            auto product_line = "D400";
            add_device(serial, product_line, available_devices, verbose);
        }
    }
}

rs2::pipeline rs2wrapper::initialize_pipeline()
{
    if (args.network())
    {
        rs2::pipeline pipe(*ctx);
        return pipe;
    }
    else
    {
        rs2::pipeline pipe;
        return pipe;
    }
}

void rs2wrapper::configure_color_stream_config(const std::string &device_sn)
{
    stream_config_colors[device_sn].stream_type = RS2_STREAM_COLOR;
    stream_config_colors[device_sn].width = args.width();
    stream_config_colors[device_sn].height = args.height();
    stream_config_colors[device_sn].format = args.color_format();
    stream_config_colors[device_sn].framerate = args.fps();
}

void rs2wrapper::configure_depth_stream_config(const std::string &device_sn)
{
    stream_config_depths[device_sn].stream_type = RS2_STREAM_DEPTH;
    stream_config_depths[device_sn].width = args.width();
    stream_config_depths[device_sn].height = args.height();
    stream_config_depths[device_sn].format = args.depth_format();
    stream_config_depths[device_sn].framerate = args.fps();
}

void rs2wrapper::configure_stream(const std::string &device_sn)
{
    if (stream_config_colors.find(device_sn) == stream_config_colors.end())
        throw std::invalid_argument("color config not found for " + device_sn);

    if (stream_config_depths.find(device_sn) == stream_config_depths.end())
        throw std::invalid_argument("depth config not found for " + device_sn);

    rs2::config cfg;
    cfg.enable_stream(stream_config_colors[device_sn].stream_type,
                      stream_config_colors[device_sn].width,
                      stream_config_colors[device_sn].height,
                      stream_config_colors[device_sn].format,
                      stream_config_colors[device_sn].framerate);
    cfg.enable_stream(stream_config_depths[device_sn].stream_type,
                      stream_config_depths[device_sn].width,
                      stream_config_depths[device_sn].height,
                      stream_config_depths[device_sn].format,
                      stream_config_depths[device_sn].framerate);

    if (!args.network())
        cfg.enable_device(std::string(device_sn));

    if (verbose)
        if (cfg.can_resolve(*enabled_devices[device_sn]->pipeline))
            print("'cfg' usable with 'pipeline' : True", 0);
        else
            print("'cfg' usable with 'pipeline' : False", 2);

    rs_cfg[device_sn] = cfg;
}

void rs2wrapper::configure_color_sensor(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];

    if (args.autoexposure())
    {
        dev->color_sensor->set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
        dev->color_sensor->set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
        // https://github.com/IntelRealSense/librealsense/issues/4015
        // Set the Auto Exposure (AE) Region of Interest (ROI). Should be done after
        // starting the pipe, will give an error otherwise
        // Create the ROI for auto exposure (set these values to whatever you need)
        // rs2::region_of_interest roi;
        // roi.min_x = 124;
        // roi.min_y = 350;
        // roi.max_x = 724;
        // roi.max_y = 450;
        // rs2::roi_sensor roi_sensor = css.as<rs2::roi_sensor>();
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // roi_sensor.set_region_of_interest(roi);
        // if (verbose)
        //     print("color sensor roi :\t" +
        //               std::to_string(roi.min_x) + "\t" +
        //               std::to_string(roi.min_y) + "\t" +
        //               std::to_string(roi.max_x) + "\t" +
        //               std::to_string(roi.max_y),
        //           0);
    }
    else
    {
        dev->color_sensor->set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
        dev->color_sensor->set_option(RS2_OPTION_EXPOSURE, 100.0);
        if (verbose)
            print("no AE for color sensor...", 0);
    }
}

void rs2wrapper::configure_depth_sensor(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];

    if (args.autoexposure())
    {
        dev->depth_sensor->set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
        auto limit = dev->depth_sensor->get_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT);
        if (verbose)
            print("depth sensor auto exposure limit : " + std::to_string(limit), 0);

        // https://github.com/IntelRealSense/librealsense/issues/4015
        // Set the Auto Exposure (AE) Region of Interest (ROI). Should be done after
        // starting the pipe, will give an error otherwise
        // Create the ROI for auto exposure (set these values to whatever you need)
        // rs2::region_of_interest roi;
        // roi.min_x = 124;
        // roi.min_y = 350;
        // roi.max_x = 724;
        // roi.max_y = 450;
        // rs2::roi_sensor roi_sensor = dss.as<rs2::roi_sensor>();
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // roi_sensor.set_region_of_interest(roi);
        // if (verbose)
        //     print("depth sensor roi :\t" +
        //               std::to_string(roi.min_x) + "\t" +
        //               std::to_string(roi.min_y) + "\t" +
        //               std::to_string(roi.max_x) + "\t" +
        //               std::to_string(roi.max_y),
        //           0);
    }
    else
    {
        dev->depth_sensor->set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
        dev->depth_sensor->set_option(RS2_OPTION_EXPOSURE, 1000.0);
        if (verbose)
            print("no AE for depth sensor...", 0);
    }
}

void rs2wrapper::configure_color_depth_sensor(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];

    std::vector<rs2::sensor> sensors =
        dev->pipeline_profile->get_device().query_sensors();

    for (auto &&sensor : sensors)
    {
        if (auto css = sensor.as<rs2::color_sensor>())
        {
            if (verbose)
                print("color sensor available...", 0);
            dev->color_sensor = std::make_shared<rs2::color_sensor>(css);
            configure_color_sensor(device_sn);
        }
        else if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
        {
            if (verbose)
                print("depth sensor available...", 0);
            dev->depth_sensor = std::make_shared<rs2::depth_stereo_sensor>(dss);
            configure_depth_sensor(device_sn);
        }
    }
}

void rs2wrapper::configure_ir_emitter(const std::string &device_sn)
{
    if (!check_if_device_is_enabled(device_sn, __func__))
        return;

    auto dss_p = enabled_devices[device_sn]->depth_sensor;

    if (dss_p->supports(RS2_OPTION_EMITTER_ENABLED))
    {
        if (args.enable_ir_emitter())
        {
            dss_p->set_option(RS2_OPTION_EMITTER_ENABLED, 1);
            auto range = dss_p->get_option_range(RS2_OPTION_LASER_POWER);
            float power = 0.0;
            if ((float)args.ir_emitter_power() > range.max)
                power = range.max;
            else if ((float)args.ir_emitter_power() < range.min)
                power = range.min;
            else
                power = (float)args.ir_emitter_power();
            dss_p->set_option(RS2_OPTION_LASER_POWER, power);
            // dss_p->set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
            if (verbose)
                print("ir emitter enabled...", 0);
        }
        else
        {
            dss_p->set_option(RS2_OPTION_EMITTER_ENABLED, 0);
            if (verbose)
                print("ir emitter not enabled...", 0);
        }
    }
    else
    {
        if (verbose)
            print("ir emitter not supported...", 1);
    }
}

bool rs2wrapper::process_color_stream(const std::string &device_sn,
                                      const rs2::frameset &frameset,
                                      const int64_t &global_timestamp,
                                      rs2_metadata_type &timestamp)
{
    try
    {
        std::shared_ptr<device> dev = enabled_devices[device_sn];

        rs2::frame frame = frameset.first_or_default(RS2_STREAM_COLOR);

        // Setting reset flag if timestamp is frozen or not valid.
        timestamp = get_frame_timestamp(device_sn, frame);
        if (timestamp == -1)
        {
            if (verbose)
                print(device_sn + " Resetting, no color timestamp", 1);
            return false;
        }
        else if (dev->color_timestamp == timestamp)
        {
            if (verbose)
                print(device_sn + " Resetting, same color timestamp", 1);
            return false;
        }

        std::string filename = pad_zeros(std::to_string(global_timestamp), 20);

        // Record per-frame metadata for UVC streams
        std::string csv_file = storagepaths[device_sn].color_metadata +
                               "/" + filename + ".csv";
        metadata_to_csv(frame, csv_file);

        // Write images to disk
        std::string png_file = storagepaths[device_sn].color +
                               "/" + filename + ".bin";
        framedata_to_bin(frame, png_file);

        // Save timestamp
        dev->color_timestamp = timestamp;

        return true;
    }
    catch (const rs2::error &e)
    {
        std::string _msg = device_sn + " :: " +
                           e.get_failed_function() +
                           "(" + e.get_failed_args() + "): " +
                           e.what();
        print(_msg, 2);
        return false;
    }
    catch (const std::exception &e)
    {
        std::string _msg = e.what();
        print(e.what(), 2);
        return false;
    }
}

bool rs2wrapper::process_depth_stream(const std::string &device_sn,
                                      const rs2::frameset &frameset,
                                      const int64_t &global_timestamp,
                                      rs2_metadata_type &timestamp)
{
    try
    {
        std::shared_ptr<device> dev = enabled_devices[device_sn];

        rs2::frame frame = frameset.first_or_default(RS2_STREAM_DEPTH);

        // Setting reset flag if timestamp is frozen or not valid.
        timestamp = get_frame_timestamp(device_sn, frame);
        if (timestamp == -1)
        {
            if (verbose)
                print(device_sn + " Resetting, no depth timestamp", 1);
            return false;
        }
        else if (dev->depth_timestamp == timestamp)
        {
            if (verbose)
                print(device_sn + " Resetting, same depth timestamp", 1);
            return false;
        }

        std::string filename = pad_zeros(std::to_string(global_timestamp), 20);

        // Record per-frame metadata for UVC streams
        std::string csv_file = storagepaths[device_sn].depth_metadata +
                               "/" + filename + ".csv";
        metadata_to_csv(frame, csv_file);

        // Write images to disk
        std::string png_file = storagepaths[device_sn].depth +
                               "/" + filename + ".bin";
        framedata_to_bin(frame, png_file);

        // Save timestamp
        dev->depth_timestamp = timestamp;

        return true;
    }
    catch (const rs2::error &e)
    {
        std::string _msg = device_sn + " :: " +
                           e.get_failed_function() +
                           "(" + e.get_failed_args() + "): " +
                           e.what();
        print(_msg, 2);
        return false;
    }
    catch (const std::exception &e)
    {
        std::string _msg = e.what();
        print(e.what(), 2);
        return false;
    }
}

int rs2wrapper::process_color_depth_stream(const std::string &device_sn,
                                           const rs2::frameset &frameset,
                                           const int64_t &global_timestamp,
                                           rs2_metadata_type &color_timestamp,
                                           rs2_metadata_type &depth_timestamp)
{
    int error_status = 0;
    if (!process_color_stream(device_sn, frameset,
                              global_timestamp, color_timestamp))
        error_status += 1;
    print_camera_temperature(device_sn);
    if (!process_depth_stream(device_sn, frameset,
                              global_timestamp, depth_timestamp))
        error_status += 2;
    return error_status;
}

bool rs2wrapper::align_frameset(const std::string &device_sn,
                                rs2::frameset &frameset,
                                rs2::frameset &aligned_frameset,
                                const int &mode)
{
    try
    {
        if (mode == 1)
        {
            aligned_frameset = align_to_color.process(frameset);
            return true;
        }
        else if (mode == 2)
        {
            aligned_frameset = align_to_depth.process(frameset);
            return true;
        }
        else
        {
            return false;
        }
    }
    catch (const rs2::error &e)
    {
        std::string _msg = device_sn + " :: " +
                           e.get_failed_function() +
                           "(" + e.get_failed_args() + "): " +
                           e.what();
        print(_msg, 2);
        return false;
    }
    catch (const std::exception &e)
    {
        print(e.what(), 2);
        return false;
    }
}

void rs2wrapper::query_timestamp_mode(const std::string &device_sn)
{
    for (auto &&frame : enabled_devices[device_sn]->pipeline->wait_for_frames())
    {
        if (auto vf = frame.as<rs2::video_frame>())
        {
            if (vf.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP))
            {
                if (verbose)
                    print("using RS2_FRAME_METADATA_SENSOR_TIMESTAMP", 0);
                timestamp_mode = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
            }
            else if (vf.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
            {
                if (verbose)
                    print("using RS2_FRAME_METADATA_FRAME_TIMESTAMP", 0);
                timestamp_mode = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
            }
            else if (vf.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
            {
                if (verbose)
                    print("using RS2_FRAME_METADATA_TIME_OF_ARRIVAL", 0);
                timestamp_mode = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
            }
            break;
        }
    }
}

void rs2wrapper::save_timestamp(const std::string &device_sn,
                                const int64_t &global_timestamp,
                                const rs2_metadata_type &color_timestamp,
                                const rs2_metadata_type &depth_timestamp)
{
    auto _path = storagepaths[device_sn].timestamp + "/timestamp.txt";
    std::fstream filestream;
    filestream.open(_path, std::fstream::in | std::fstream::out | std::fstream::app);
    // If file does not exist, Create new file
    if (!filestream)
    {
        filestream.open(_path, std::fstream::in | std::fstream::out | std::fstream::trunc);
    }
    filestream << global_timestamp
               << "::"
               << color_timestamp
               << "::"
               << depth_timestamp
               << "\n";
    filestream.close();
}

void rs2wrapper::print_camera_infos(const std::shared_ptr<rs2::pipeline_profile> profile)
{
    if (!verbose)
        return;

    std::cout << std::string(80, '=') << std::endl;
    std::cout << ">>>>> RS2_CAMERA_INFO <<<<<" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    std::cout << "Name          : " << profile->get_device().get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << "Serial Number : " << profile->get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    try
    {
        std::cout << "Product Line  : " << profile->get_device().get_info(RS2_CAMERA_INFO_PRODUCT_LINE) << std::endl;
    }
    catch (const rs2::error &e)
    {
        std::cout << "not available, " << e.what() << std::endl;
    }
    try
    {
        std::cout << "Firmware      : " << profile->get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    }
    catch (const rs2::error &e)
    {
        std::cout << "not available, " << e.what() << std::endl;
    }
    try
    {
        std::cout << "USB type      : " << profile->get_device().get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
    }
    catch (const rs2::error &e)
    {
        std::cout << "not available, " << e.what() << std::endl;
    }
    std::cout << std::string(80, '=') << std::endl;
}

void rs2wrapper::print_camera_temperature(const std::string &device_sn)
{
    if (!verbose)
        return;

    int c = enabled_devices[device_sn]->camera_temp_printout_counter;
    if (c >= camera_temp_printout_interval || c == -1)
    {
        enabled_devices[device_sn]->camera_temp_printout_counter = 0;
        auto dss = enabled_devices[device_sn]->depth_sensor;
        if (dss->supports(RS2_OPTION_ASIC_TEMPERATURE))
        {
            auto temp = dss->get_option(RS2_OPTION_ASIC_TEMPERATURE);
            print(device_sn + " Temperature ASIC      : " + std::to_string(temp), 0);
        }
        if (dss->supports(RS2_OPTION_PROJECTOR_TEMPERATURE))
        {
            auto temp = dss->get_option(RS2_OPTION_PROJECTOR_TEMPERATURE);
            print(device_sn + " Temperature Projector : " + std::to_string(temp), 0);
        }
    }
    enabled_devices[device_sn]->camera_temp_printout_counter += 1;
}
