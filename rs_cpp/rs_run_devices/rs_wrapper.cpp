#include "rs_wrapper.hpp"

std::mutex reset_mux;

bool framedata_to_bin(const rs2::frame &frm, const std::string &filename)
{
    bool ret = false;
    rs2::video_frame image = frm.as<rs2::video_frame>();
    if (image)
    {
        std::ofstream outfile(filename, std::ofstream::binary);
        outfile.write(static_cast<const char *>(image.get_data()),
                      image.get_height() * image.get_stride_in_bytes());
        outfile.close();
        ret = true;
    }
    return ret;
}

void metadata_to_csv(const rs2::frame &frm, const std::string &filename)
{
    std::ofstream csv;
    csv.open(filename);

    csv << "Stream,"
        << rs2_stream_to_string(frm.get_profile().stream_type())
        << "\nAttribute,Value\n";

    rs2_frame_metadata_value metadata_idx;
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        metadata_idx = (rs2_frame_metadata_value)i;
        if (frm.supports_frame_metadata(metadata_idx))
        {
            // rs2_metadata_type => long long.
            rs2_metadata_type metadata = frm.get_frame_metadata(metadata_idx);
            csv << rs2_frame_metadata_to_string(metadata_idx)
                << ","
                << metadata
                << "\n";
        }
    }
    csv.close();
}

storagepaths::storagepaths()
{
}

void storagepaths::create(const std::string &device_sn,
                          const std::string &base_path)
{
    std::string path;
    // Base
    path = base_path;
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Device
    path.append("/");
    path.append(device_sn);
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // System time , trial
    time_t current_time;
    time(&current_time);
    path.append("/");
    path.append(std::to_string(current_time));
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Calib
    calib = path + "/calib";
    mkdir(calib.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Timestamp
    timestamp = path + "/timestamp";
    mkdir(timestamp.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Color
    color = path + "/color";
    mkdir(color.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    color_metadata = path + "/color_metadata";
    mkdir(color_metadata.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Depth
    depth = path + "/depth";
    mkdir(depth.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    depth_metadata = path + "/depth_metadata";
    mkdir(depth_metadata.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void storagepaths::show()
{
    print("calib : " + calib, 0);
    print("color : " + color, 0);
    print("depth : " + depth, 0);
    print("color_metadata : " + color_metadata, 0);
    print("depth_metadata : " + depth_metadata, 0);
}

/*******************************************************************************
 * rs2wrapper PUBLIC FUNCTIONS
 ******************************************************************************/
rs2wrapper::rs2wrapper(int argc,
                       char *argv[],
                       rs2::context context,
                       std::string device_sn)
{
    // CLI args
    args = rs2args(argc, argv);

    // prints out CLI args
    args.print_args();

    // if arg is given, we use only one rs device
    single_device_sn = device_sn;

    // context grabs the usb resources of the cameras.
    // ctx = std::make_shared<rs2::context>(context);

    // Get available devices
    if (args.network())
    {
        ctx = std::make_shared<rs2::context>(context);
        print("Network mode", 0);
        rs2::net_device dev(args.ip());
        print("Network device found", 0);
        dev.add_to(*ctx);
        auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::vector<std::string> available_device;
        available_device.push_back(serial);
        available_device.push_back(args.ip());
        available_devices.push_back(available_device);
        print("using : " + std::string(serial), 0);
    }
    else
    {
        print("Local mode", 0);
        if (single_device_sn == "-1")
        {
            ctx = std::make_shared<rs2::context>(context);
            for (auto &&dev : ctx->query_devices())
            {
                auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                auto product_line = dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
                std::vector<std::string> available_device;
                available_device.push_back(serial);
                available_device.push_back(product_line);
                available_devices.push_back(available_device);
                print("found : " + std::string(serial), 0);
            }
        }
        else
        {
            std::vector<std::string> available_device;
            available_device.push_back(single_device_sn.c_str());
            available_device.push_back("D400");
            available_devices.push_back(available_device);
            print("using : " + std::string(single_device_sn.c_str()), 0);
        }
    }

    // Sort the devices.
    std::sort(available_devices.begin(), available_devices.end(),
              [](const std::vector<std::string> &a,
                 const std::vector<std::string> &b)
              {
                  return a[0] < b[0];
              });

    // get available devices_sn.
    for (auto &&available_device : available_devices)
        available_devices_sn.push_back(available_device[0]);

    // storage
    for (auto &&device_sn : available_devices_sn)
    {
        storagepaths _storagepaths;
        _storagepaths.create(device_sn, args.save_path());
        storagepaths_perdev[device_sn] = _storagepaths;
    }

    // stream
    set_color_stream_config(args.width(), args.height(), args.fps(),
                            args.color_format());
    set_depth_stream_config(args.width(), args.height(), args.fps(),
                            args.depth_format());
}

void rs2wrapper::initialize(const bool &enable_ir_emitter,
                            const bool &verbose)
{
    for (auto &&device_sn : available_devices_sn)
    {
        initialize(device_sn, enable_ir_emitter, verbose);
    }
}

void rs2wrapper::initialize(const std::string &device_sn,
                            const bool &enable_ir_emitter,
                            const bool &verbose)
{
    print("Initializing RealSense devices " + std::string(device_sn), 0);

    // 0. enabled devices
    std::shared_ptr<device> dev = std::make_shared<device>();
    enabled_devices[device_sn] = dev;
    reset_flags[device_sn] = false;

    // 1. pipeline
    rs2::pipeline pipe = initialize_pipeline();
    dev->pipeline = std::make_shared<rs2::pipeline>(pipe);

    // 2. configure
    configure_stream(device_sn, stream_config_color, stream_config_depth);
    rs2::config cfg = rs_cfg[device_sn];
    if (!args.network())
        cfg.enable_device(std::string(device_sn));
    bool check = cfg.can_resolve(pipe);
    if (verbose)
        if (check)
            print("'cfg' usable with 'pipeline' : True", 0);
        else
            print("'cfg' usable with 'pipeline' : False", 0);
    rs_cfg[device_sn] = cfg;

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
        if (auto css = sensor.as<rs2::color_sensor>())
        {
            dev->color_sensor = std::make_shared<rs2::color_sensor>(css);
            if (verbose)
                print("color sensor available...", 0);
        }
        else if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
        {
            dev->depth_sensor = std::make_shared<rs2::depth_stereo_sensor>(dss);
            if (verbose)
                print("depth sensor available...", 0);
        }
    }

    // 5. IR
    if (enable_ir_emitter)
    {
        if (dev->depth_sensor->supports(RS2_OPTION_EMITTER_ENABLED))
        {
            // TODO: add arg for this.
            dev->depth_sensor->set_option(RS2_OPTION_EMITTER_ENABLED, 0);
            if (verbose)
                print("ir emitter enabled...", 0);
        }
    }

    // 6. infos
    if (verbose)
    {
        print_camera_infos(dev->pipeline_profile);
        print_camera_temperature(device_sn);
    }

    if (storagepaths_perdev.size() > 0)
        query_timestamp_mode(std::string(device_sn));

    // 7. get enabled devices_sn.
    enabled_devices_sn.push_back(device_sn);

    print("Initialized RealSense devices " + std::string(device_sn), 0);
}

void rs2wrapper::start()
{
    if (enabled_devices.size() > 0)
        for (auto &&device_sn : enabled_devices_sn)
            start(device_sn);
    else
        print("no device has not been enabled, skipping stop()...", 1);
}

void rs2wrapper::start(const std::string &device_sn)
{
    if (!check_enabled_device(device_sn, __func__))
        return;

    rs2::config cfg = rs_cfg[device_sn];
    std::shared_ptr<device> dev = enabled_devices[device_sn];
    rs2::pipeline_profile profile = dev->pipeline->start(cfg);
    dev->pipeline_profile = std::make_shared<rs2::pipeline_profile>(profile);
    dev->num_streams = dev->pipeline_profile->get_streams().size();
    print(device_sn + " has been started...", 0);
}

void rs2wrapper::step_clear()
{
    output_msg_list.clear();
    valid_frame_check_flag.clear();
    empty_frame_check_counter.clear();
}

bool rs2wrapper::step_receiving_frame_from_all_devices()
{
    return valid_frame_check_flag.size() < enabled_devices.size();
}

void rs2wrapper::step()
{
    step_clear();

    while (step_receiving_frame_from_all_devices())
    {
        for (auto &&device_sn : enabled_devices_sn)
        {
            step(device_sn);
            reset_with_high_reset_counter(device_sn);
            // In case a device is not sending anything at all.
            // empty frame for 1 seconds.
            if (empty_frame_check_counter[device_sn] > 1000000000)
            {
                set_valid_frame_check_flag(device_sn, false);
                reset(device_sn);
            }
        }
    }

    // reset_with_high_reset_counter();
}

void rs2wrapper::step(const std::string &device_sn)
{
    reset_flags[device_sn] = false;

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
        int64_t timestamp_diff = get_timestamp_duration_ns(local_timestamp);
        empty_frame_check_counter[device_sn] += timestamp_diff;
    }
    // Polled frames are not empty.
    else
    {
        // Make sure both color and depth frames are there.
        if (frameset.size() == dev->num_streams)
        {
            // Timestamp duration using the global start timestamp.
            int64_t global_timestamp_diff = get_timestamp_duration_ns(
                global_timestamp_start);

            // Check if both frames are valid, skip step if one is invalid.
            if (!check_valid_color_depth_streams(device_sn, frameset))
            {
                dev->color_reset_counter += 1;
                dev->depth_reset_counter += 1;
                reset_flags[device_sn] = true;
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
                    reset_flags[device_sn] = true;
                    output_msg = device_sn + " :: Align failed...";
                }
                // Framesets are aligned.
                else
                {
                    // Loops through the streams to get color and depth.
                    // This is needed for multi cam setup.
                    bool error_flag = process_color_depth_stream(
                        device_sn, aligned_frameset, current_color_timestamp,
                        current_depth_timestamp);

                    // Saves the timestamps and generate output message.
                    if (!error_flag)
                    {
                        save_timestamp(device_sn,
                                       global_timestamp_diff,
                                       current_color_timestamp,
                                       current_depth_timestamp);

                        set_valid_frame_check_flag(device_sn, true);
                        empty_frame_check_counter[device_sn] = 0;

                        output_msg =
                            device_sn + "::" +
                            std::to_string(global_timestamp_diff) + "::" +
                            std::to_string(current_color_timestamp) + "::" +
                            std::to_string(current_depth_timestamp) + "  ";
                    }
                    // Something was wrong with color/depth stream.
                    // No timestamp is saved, and an error message is generated.
                    else
                    {
                        output_msg = device_sn + " :: Error in stream...";
                    }
                }
            }
        }
    }

    // Message to be printed out.
    std::pair<std::string, std::string> msg(device_sn, output_msg);
    output_msg_list.push_back(msg);
}

void rs2wrapper::stop()
{
    if (enabled_devices.size() > 0)
        for (auto &&device_sn : enabled_devices_sn)
            stop(device_sn);
    else
        print("no device has not been enabled, skipping stop()...", 1);
}

void rs2wrapper::stop(const std::string &device_sn)
{
    if (!check_enabled_device(device_sn, __func__))
        return;

    enabled_devices[device_sn]->pipeline->stop();
    print(device_sn + " has been stopped...", 0);
}

void rs2wrapper::reset()
{
    if (enabled_devices.size() > 0)
        for (auto &&device_sn : enabled_devices_sn)
            reset(device_sn);
    else
        print("no device has not been enabled, skipping reset()...", 1);
}

void rs2wrapper::reset(const std::string &device_sn)
{
    if (!check_enabled_device(device_sn, __func__))
        return;

    std::lock_guard<std::mutex> lock(reset_mux);
    stop(device_sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    start(device_sn);
    print(device_sn + " pipeline has been restarted with 100ms sleep...", 0);
}

void rs2wrapper::reset_with_high_reset_counter()
{
    if (enabled_devices.size() > 0)
        for (auto &&device_sn : enabled_devices_sn)
            reset_with_high_reset_counter(device_sn);
    else
        print("no device has not been enabled, skipping reset_with_high_reset_counter()...", 1);
}

void rs2wrapper::reset_with_high_reset_counter(const std::string &device_sn)
{
    if (!check_enabled_device(device_sn, __func__))
        return;

    if (reset_flags[device_sn])
    {
        if (enabled_devices[device_sn]->color_reset_counter > max_reset_counter)
        {
            print("Reset " + device_sn + " due to high color stream reset counter", 1);
            reset(device_sn);
            enabled_devices[device_sn]->color_reset_counter = 0;
            enabled_devices[device_sn]->depth_reset_counter = 0;
        }
        if (enabled_devices[device_sn]->depth_reset_counter > max_reset_counter)
        {
            print("Reset " + device_sn + " due to high depth stream reset counter", 1);
            reset(device_sn);
            enabled_devices[device_sn]->color_reset_counter = 0;
            enabled_devices[device_sn]->depth_reset_counter = 0;
        }
    }
}

void rs2wrapper::save_calib()
{
    if (enabled_devices.size() > 0)
        for (auto &&device_sn : enabled_devices_sn)
            save_calib(device_sn);
    else
        print("no device has not been enabled, skipping save_calib()...", 1);
}

void rs2wrapper::save_calib(const std::string &device_sn)
{
    if (!check_enabled_device(device_sn, __func__))
        return;

    std::shared_ptr<device> dev = enabled_devices[device_sn];
    std::string csv_file = storagepaths_perdev[device_sn].calib + "/calib.csv";
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
    for (auto &&value : intr_color.coeffs)
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
    for (auto &&value : intr_depth.coeffs)
        csv << value << ",";
    csv << args.depth_format() << ","
        << args.fps() << ",";
    csv << "\n";

    for (auto &&value : extr.rotation)
        csv << value << ",";
    for (auto &&value : extr.translation)
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
    if (enabled_devices.size() > 0)
        for (auto &&device_sn : enabled_devices_sn)
            flush_frames(device_sn, num_frames);
    else
        print("no device has not been enabled, skipping flush_frames()...", 1);
}

void rs2wrapper::flush_frames(const std::string &device_sn,
                              const int &num_frames)
{
    if (!check_enabled_device(device_sn, __func__))
        return;

    int _num_frames = 0;
    if (num_frames == -1)
        _num_frames = args.flush_steps();
    else
        _num_frames = num_frames;

    std::shared_ptr<device> dev = enabled_devices[device_sn];
    for (auto i = 0; i < _num_frames; ++i)
        dev->pipeline->wait_for_frames();
    print(device_sn + " Flushed " + std::to_string(_num_frames) + " initial frames...", 0);
}

void rs2wrapper::reset_global_timestamp()
{
    this->global_timestamp_start = std::chrono::steady_clock::now();
}

void rs2wrapper::reset_global_timestamp(std::chrono::steady_clock::time_point global_timestamp)
{
    this->global_timestamp_start = global_timestamp;
}

void rs2wrapper::set_color_stream_config(const int &width, const int &height,
                                         const int &fps, const rs2_format &format)
{
    stream_config_color.stream_type = RS2_STREAM_COLOR;
    stream_config_color.width = width;
    stream_config_color.height = height;
    stream_config_color.format = format;
    stream_config_color.framerate = fps;
}

void rs2wrapper::set_depth_stream_config(const int &width, const int &height,
                                         const int &fps, const rs2_format &format)
{
    stream_config_depth.stream_type = RS2_STREAM_DEPTH;
    stream_config_depth.width = width;
    stream_config_depth.height = height;
    stream_config_depth.format = format;
    stream_config_depth.framerate = fps;
}

void rs2wrapper::set_valid_frame_check_flag(const std::string &device_sn,
                                            const bool &flag)
{
    this->valid_frame_check_flag[device_sn] = flag;
}

int64_t rs2wrapper::get_empty_frame_check_counter(const std::string &device_sn)
{
    return this->empty_frame_check_counter[device_sn];
}

std::map<std::string, int64_t> rs2wrapper::get_empty_frame_check_counter()
{
    return this->empty_frame_check_counter;
}

std::string rs2wrapper::get_output_msg()
{
    std::string output_msg;
    std::sort(this->output_msg_list.begin(), this->output_msg_list.end());
    for (auto &&_output_msg : this->output_msg_list)
        output_msg += _output_msg.second;
    return output_msg;
}

std::vector<std::vector<std::string>> rs2wrapper::get_available_devices()
{
    return this->available_devices;
}

std::vector<std::string> rs2wrapper::get_available_devices_sn()
{
    return this->available_devices_sn;
}

std::map<std::string, std::shared_ptr<device>> rs2wrapper::get_enabled_devices()
{
    return this->enabled_devices;
}

std::vector<std::string> rs2wrapper::get_enabled_devices_sn()
{
    return this->enabled_devices_sn;
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

bool rs2wrapper::check_enabled_device(const std::string &device_sn,
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

bool rs2wrapper::check_valid_color_depth_streams(const std::string &device_sn,
                                                 const rs2::frameset &frameset)
{
    rs2::frame cf = frameset.first_or_default(RS2_STREAM_COLOR);
    rs2::frame df = frameset.first_or_default(RS2_STREAM_DEPTH);
    if (!cf || !df)
        return false;
    else
        return true;
}

/*******************************************************************************
 * rs2wrapper PRIVATAE FUNCTIONS
 ******************************************************************************/
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

void rs2wrapper::query_timestamp_mode(const std::string &device_sn)
{
    for (auto &&frame : enabled_devices[device_sn]->pipeline->wait_for_frames())
    {
        if (auto vf = frame.as<rs2::video_frame>())
        {
            if (vf.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP))
            {
                print("using RS2_FRAME_METADATA_SENSOR_TIMESTAMP", 0);
                timestamp_mode = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
            }
            else if (vf.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
            {
                print("using RS2_FRAME_METADATA_FRAME_TIMESTAMP", 0);
                timestamp_mode = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
            }
            else if (vf.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL))
            {
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
    auto _path = storagepaths_perdev[device_sn].timestamp + "/timestamp.txt";
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

void rs2wrapper::configure_stream(const std::string &device_sn,
                                  const stream_config &stream_config_color,
                                  const stream_config &stream_config_depth)
{
    rs2::config cfg;
    cfg.enable_stream(stream_config_color.stream_type,
                      stream_config_color.width,
                      stream_config_color.height,
                      stream_config_color.format,
                      stream_config_color.framerate);
    cfg.enable_stream(stream_config_depth.stream_type,
                      stream_config_depth.width,
                      stream_config_depth.height,
                      stream_config_depth.format,
                      stream_config_depth.framerate);
    rs_cfg[device_sn] = cfg;
}

bool rs2wrapper::process_color_stream(const std::string &device_sn,
                                      const rs2::frameset &frameset,
                                      rs2_metadata_type &timestamp)
{
    try
    {
        rs2::frame frame = frameset.first_or_default(RS2_STREAM_COLOR);
        timestamp = get_frame_timestamp(device_sn, frame);
        std::string filename = pad_zeros(std::to_string(timestamp), 12);
        // Record per-frame metadata for UVC streams
        std::string csv_file =
            storagepaths_perdev[device_sn].color_metadata + "/" + filename + ".csv";
        metadata_to_csv(frame, csv_file);
        // Write images to disk
        std::string png_file =
            storagepaths_perdev[device_sn].color + "/" + filename + ".bin";
        framedata_to_bin(frame, png_file);
        // Release frame
        // TODO: WORKED????
        rs2_release_frame(frame.get());
        // Setting reset flag if timestamp is frozen
        {
            std::shared_ptr<device> dev = enabled_devices[device_sn];
            if (timestamp == -1)
            {
                dev->color_timestamp = timestamp;
            }
            else if (dev->color_timestamp == timestamp)
            {
                dev->color_reset_counter += 1;
                reset_flags[device_sn] = true;
                std::string c_msg = std::to_string(dev->color_reset_counter);
                print(device_sn + " Resetting, same color timestamp, c=" + c_msg, 1);
            }
            else
            {
                dev->color_timestamp = timestamp;
            }
        }
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
        print(e.what(), 2);
        return false;
    }
}

bool rs2wrapper::process_depth_stream(const std::string &device_sn,
                                      const rs2::frameset &frameset,
                                      rs2_metadata_type &timestamp)
{
    try
    {
        rs2::frame frame = frameset.first_or_default(RS2_STREAM_DEPTH);
        timestamp = get_frame_timestamp(device_sn, frame);
        std::string filename = pad_zeros(std::to_string(timestamp), 12);
        // Record per-frame metadata for UVC streams
        std::string csv_file =
            storagepaths_perdev[device_sn].depth_metadata + "/" + filename + ".csv";
        metadata_to_csv(frame, csv_file);
        // Write images to disk
        std::string png_file =
            storagepaths_perdev[device_sn].depth + "/" + filename + ".bin";
        framedata_to_bin(frame, png_file);
        // Release frame
        // TODO: WORKED????
        rs2_release_frame(frame.get());
        // Setting reset flag if timestamp is frozen
        {
            std::shared_ptr<device> dev = enabled_devices[device_sn];
            if (timestamp == -1)
            {
                dev->depth_timestamp = timestamp;
            }
            else if (dev->depth_timestamp == timestamp)
            {
                dev->depth_reset_counter += 1;
                reset_flags[device_sn] = true;
                std::string c_msg = std::to_string(dev->depth_reset_counter);
                print(device_sn + " Resetting, same depth timestamp, c=" + c_msg, 1);
            }
            else
            {
                dev->depth_timestamp = timestamp;
            }
        }
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
        print(e.what(), 2);
        return false;
    }
}

bool rs2wrapper::process_color_depth_stream(const std::string &device_sn,
                                            const rs2::frameset &frameset,
                                            rs2_metadata_type &color_timestamp,
                                            rs2_metadata_type &depth_timestamp)
{
    bool error_flag = false;
    std::shared_ptr<device> dev = enabled_devices[device_sn];
    std::vector<rs2::stream_profile> streams = dev->pipeline_profile->get_streams();
    for (auto &&stream : streams)
    {
        if (stream.stream_type() == RS2_STREAM_COLOR)
        {
            if (!process_color_stream(device_sn, frameset, color_timestamp))
            {
                dev->color_reset_counter += 1;
                reset_flags[device_sn] = true;
                error_flag = true;
                break;
            }
        }
        else if (stream.stream_type() == RS2_STREAM_DEPTH)
        {
            print_camera_temperature(device_sn);
            if (!process_depth_stream(device_sn, frameset, depth_timestamp))
            {
                dev->depth_reset_counter += 1;
                reset_flags[device_sn] = true;
                error_flag = true;
                break;
            }
        }
    }
    return error_flag;
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

void rs2wrapper::print_camera_infos(const std::shared_ptr<rs2::pipeline_profile> profile)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << ">>>>> RS2_CAMERA_INFO <<<<<" << std::endl;
    std::cout << "========================================" << std::endl;
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
    std::cout << "========================================\n"
              << std::endl;
}

void rs2wrapper::print_camera_temperature(const std::string &device_sn)
{
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
