#include "utils.h"
#include "rs_wrapper.h"

// doesnt work in the class init.
rs2::align align_to_depth(RS2_STREAM_DEPTH);
rs2::align align_to_color(RS2_STREAM_COLOR);

int num_zeros_to_pad = NUM_ZEROS_TO_PAD;

void storagepaths::create_directories(const char *device_sn,
                                      const char *base_path)
{
    std::string path;
    // Base
    path = std::string(base_path);
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Device
    path.append("/");
    path.append(device_sn);
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // System time
    time_t current_time;
    time(&current_time);
    path.append("/");
    path.append(std::to_string(current_time));
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Calib
    calib = path + "/Calib";
    mkdir(calib.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Color
    color = path + "/Color";
    mkdir(color.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    color_metadata = path + "/MetaColor";
    mkdir(color_metadata.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Depth
    depth = path + "/Depth";
    mkdir(depth.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    depth_metadata = path + "/MetaDepth";
    mkdir(depth_metadata.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

rs2wrapper::rs2wrapper(int argc,
                       char *argv[],
                       rs2::context context,
                       std::string device_sn) : rs2args(argc, argv)
{
    // prints out CLI args
    print_args();

    single_device_sn = device_sn;

    // context grabs the usb resources of the cameras.
    ctx = std::make_shared<rs2::context>(context);

    // stream
    stream_config_color.stream_type = RS2_STREAM_COLOR;
    stream_config_color.width = width();
    stream_config_color.height = height();
    stream_config_color.format = color_format();
    stream_config_color.framerate = fps();
    stream_config_depth.stream_type = RS2_STREAM_DEPTH;
    stream_config_depth.width = width();
    stream_config_depth.height = height();
    stream_config_depth.format = depth_format();
    stream_config_depth.framerate = fps();
}

rs2::pipeline rs2wrapper::initialize_pipeline(const std::shared_ptr<rs2::context> context)
{
    if (network())
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

void rs2wrapper::initialize(bool enable_ir_emitter, bool verbose)
{
    // Get available devices
    if (network())
    {
        print("Network mode", 0);
        rs2::net_device dev((std::string)ip());
        print("Network device found", 0);
        dev.add_to(*ctx);
        // pipe = rs2::pipeline(ctx);
        std::vector<const char *> available_device;
        available_device.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        available_device.push_back(ip());
        available_devices.push_back(available_device);
    }
    else
    {
        print("Local mode", 0);
        if (single_device_sn != "-1")
        {
            for (auto &&dev : ctx->query_devices())
            {
                auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                auto product_line = dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
                std::vector<const char *> available_device;
                available_device.push_back(serial);
                available_device.push_back(product_line);
                available_devices.push_back(available_device);
            }
        }
        else
        {
            std::vector<const char *> available_device;
            available_device.push_back(single_device_sn.c_str());
            available_device.push_back("D400");
            available_devices.push_back(available_device);
        }
    }

    // Sort the devices.
    std::sort(available_devices.begin(), available_devices.end(),
              [](const std::vector<const char *> &a,
                 const std::vector<const char *> &b)
              {
                  return a[0] < b[0];
              });

    // storage
    for (auto &&available_device : available_devices)
    {
        storagepaths _storagepaths;
        _storagepaths.create_directories(available_device[0], save_path());
        storagepaths_perdev[available_device[0]] = _storagepaths;
    }

    // main init
    for (auto &&available_device : available_devices)
    {
        const char *device_sn = available_device[0];
        device dev;

        print("Initializing RealSense devices " + std::string(device_sn), 0);

        // 1. pipeline
        rs2::pipeline pipe = initialize_pipeline(ctx);
        dev.pipeline = std::make_shared<rs2::pipeline>(pipe);

        // 2. configure
        configure_stream(device_sn, stream_config_color, stream_config_depth);
        rs2::config cfg = rs_cfg[device_sn];
        if (!network())
            cfg.enable_device(device_sn);
        bool check = cfg.can_resolve(pipe);
        if (check)
            print("'cfg' usable with 'pipeline' : True", 0);
        else
            print("'cfg' usable with 'pipeline' : False", 0);

        // 3. pipeline profile
        rs2::pipeline_profile profile = pipe.start(cfg);
        dev.pipeline_profile = std::make_shared<rs2::pipeline_profile>(profile);
        if (verbose)
            print("pipeline started...", 0);

        // 4. sensors
        std::vector<rs2::sensor> sensors = profile.get_device().query_sensors();
        for (auto &&sensor : sensors)
        {
            if (auto css = sensor.as<rs2::color_sensor>())
            {
                dev.color_sensor = std::make_shared<rs2::color_sensor>(css);
                print("color sensor available...", 0);
            }
            else if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
            {
                dev.depth_sensor = std::make_shared<rs2::depth_stereo_sensor>(dss);
                print("depth sensor available...", 0);
            }
        }

        // 5. IR
        if (enable_ir_emitter)
        {
            if (dev.depth_sensor->supports(RS2_OPTION_EMITTER_ENABLED))
            {
                // TODO
                dev.depth_sensor->set_option(RS2_OPTION_EMITTER_ENABLED, 1);
                print("ir emitter enabled...", 0);
            }
        }

        // 6. enabled devices
        enabled_devices[device_sn] = dev;
        print_camera_infos(dev.pipeline_profile);

        // Create save directory
        create_directories(device_sn, save_path());

        print("Initialized RealSense devices " + std::string(device_sn), 0);
    }
}

void rs2wrapper::initial_flush(const int &num_frames)
{
    for (const auto &enabled_device : enabled_devices)
    {
        auto device_sn = enabled_device.first;
        auto dev = enabled_device.second;
        for (auto i = 0; i < num_frames; ++i)
            dev.pipeline->wait_for_frames();
        print("Flushed 30 initial frames...", 0);
    }
}

void rs2wrapper::step(const std::string &save_file_prefix)
{
    for (const auto &enabled_device : enabled_devices)
    {
        auto device_sn = enabled_device.first;
        auto dev = enabled_device.second;
        // Loop through the set of frames from the camera.
        for (auto &&frame : dev.pipeline->wait_for_frames())
        {
            // We can only save video frames as pngs, so we skip the rest
            if (auto vf = frame.as<rs2::video_frame>())
            {
                std::string prefix = filename_prefix_with_timestamp(
                    vf, save_file_prefix, num_zeros_to_pad);

                rs2::stream_profile vfsp = vf.get_profile();
                std::string stream_name = vfsp.stream_name();

                if (stream_name == "Color")
                {
                    // Record per-frame metadata for UVC streams
                    std::string csv_file =
                        storagepaths_perdev[device_sn].color_metadata + "/" +
                        prefix + ".csv";
                    metadata_to_csv(vf, csv_file);
                    // Write images to disk
                    std::string png_file =
                        storagepaths_perdev[device_sn].color + "/" +
                        prefix + ".bin";
                    framedata_to_bin(frame, png_file);
                }
                else if (stream_name == "Depth")
                {
                    // Record per-frame metadata for UVC streams
                    std::string csv_file =
                        storagepaths_perdev[device_sn].depth_metadata + "/" +
                        prefix + ".csv";
                    metadata_to_csv(vf, csv_file);
                    // Write images to disk
                    std::string png_file =
                        storagepaths_perdev[device_sn].depth + "/" +
                        prefix + ".bin";
                    framedata_to_bin(frame, png_file);
                }

                rs2_intrinsics intrinsics = vfsp.as<rs2::video_stream_profile>().get_intrinsics();
                rs2_extrinsics extrinsics = vfsp.get_extrinsics_to(vfsp);
            }
        }
    }
}

void rs2wrapper::save_calib()
{
    for (const auto &enabled_device : enabled_devices)
    {
        auto device_sn = enabled_device.first;
        auto dev = enabled_device.second;
        std::string csv_file = storagepaths_perdev[device_sn].calib + "/calib.csv";
        std::ofstream csv;
        csv.open(csv_file);

        // Intrinsics of color & depth frames
        rs2::stream_profile profile_color =
            dev.pipeline_profile->get_stream(RS2_STREAM_COLOR);
        rs2_intrinsics intr_color =
            profile_color.as<rs2::video_stream_profile>().get_intrinsics();
        // Fetch stream profile for depth stream
        // Downcast to video_stream_profile and fetch intrinsics
        rs2::stream_profile profile_depth =
            dev.pipeline_profile->get_stream(RS2_STREAM_DEPTH);
        rs2_intrinsics intr_depth =
            profile_depth.as<rs2::video_stream_profile>().get_intrinsics();

        // Extrinsic matrix from color sensor to Depth sensor
        rs2_extrinsics extr = profile_color.as<rs2::video_stream_profile>().get_extrinsics_to(profile_depth);

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
        csv << "\n";

        for (auto &&value : extr.rotation)
            csv << value << ",";
        for (auto &&value : extr.translation)
            csv << value << ",";
        csv << "\n";

        std::vector<rs2::sensor> sensors =
            dev.pipeline_profile->get_device().query_sensors();
        for (auto &&sensor : sensors)
        {
            if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
            {
                csv << dss.get_depth_scale() << ","
                    << dss.get_stereo_baseline() << ",\n";
            }
        }

        std::cout << "[INFO] : Saved camera calibration data..." << std::endl;
    }
}

void rs2wrapper::configure_stream(const char *device_sn,
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

void rs2wrapper::print_camera_infos(const std::shared_ptr<rs2::pipeline_profile> profile)
{
    std::cout << "========================================" << std::endl;
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
    std::cout << "========================================" << std::endl;
}

std::string pad_zeros(const std::string &in_str, const size_t &num_zeros)
{
    std::string out_str =
        std::string(num_zeros - std::min(num_zeros, in_str.length()), '0') +
        in_str;
    return out_str;
}

std::string filename_prefix_with_timestamp(const rs2::frame &frm,
                                           const std::string &prefix,
                                           const size_t &num_zeros)
{
    std::string _prefix;
    if (frm.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP))
    {
        _prefix = std::to_string(frm.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP));
        _prefix = pad_zeros(_prefix, num_zeros);
    }
    else if (frm.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP))
    {
        _prefix = std::to_string(frm.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));
        _prefix = pad_zeros(_prefix, num_zeros);
    }
    else
    {
        _prefix = prefix;
    }
    return _prefix;
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
