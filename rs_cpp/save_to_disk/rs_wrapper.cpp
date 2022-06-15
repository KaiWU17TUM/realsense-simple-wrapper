#include "rs_wrapper.h"

int num_zeros_to_pad = NUM_ZEROS_TO_PAD;

rs2wrapper::rs2wrapper(int argc, char *argv[]) : rs2args(argc, argv)
{
    // prints out info
    info();
    // Create save directory
    create_directories();
}

void rs2wrapper::initialize()
{
    // Add network device context
    if (network())
    {
        rs2::context ctx;
        rs2::net_device dev(ip());
        std::cout << "IP address found..." << std::endl;
        dev.add_to(ctx);
        pipe = rs2::pipeline(ctx);
    }
    // Configure pipeline config
    cfg.enable_stream(RS2_STREAM_COLOR, width(), height(), RS2_FORMAT_RGB8, fps());
    cfg.enable_stream(RS2_STREAM_DEPTH, width(), height(), RS2_FORMAT_Z16, fps());
    profile = pipe.start(cfg);
    std::cout << "Pipeline started..." << std::endl;
    std::cout << "Initialized realsense device..." << std::endl;
}

void rs2wrapper::create_directories()
{
    // Base
    mkdir(save_path(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // System time
    time_t current_time;
    time(&current_time);
    auto path = (std::string)save_path() + "/" + std::to_string(current_time);
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Calib
    path_map["Calib"] = path + "/Calib";
    mkdir(path_map["Calib"].c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Color
    path_map["Color"] = path + "/Color";
    mkdir(path_map["Color"].c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    path_map["MetaColor"] = path + "/MetaColor";
    mkdir(path_map["MetaColor"].c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Depth
    path_map["Depth"] = path + "/Depth";
    mkdir(path_map["Depth"].c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    path_map["MetaDepth"] = path + "/MetaDepth";
    mkdir(path_map["MetaDepth"].c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void rs2wrapper::initial_flush(const int &num_frames)
{
    for (auto i = 0; i < num_frames; ++i)
        pipe.wait_for_frames();
    std::cout << "Flushed 30 initial frames..." << std::endl;
}

void rs2wrapper::step(const std::string &save_file_prefix)
{
    // Loop through the set of frames from the camera.
    for (auto &&frame : pipe.wait_for_frames())
    {
        // We can only save video frames as pngs, so we skip the rest
        if (auto vf = frame.as<rs2::video_frame>())
        {
            std::string prefix = filename_prefix_with_timestamp(
                vf, save_file_prefix, num_zeros_to_pad);

            rs2::stream_profile vfsp = vf.get_profile();
            std::string stream_name = vfsp.stream_name();

            // Record per-frame metadata for UVC streams
            std::string csv_file = path_map["Meta" + stream_name] + "/" +
                                   prefix + "-metadata.csv";
            metadata_to_csv(vf, csv_file);

            // Write images to disk
            std::string png_file = path_map[stream_name] + "/" +
                                   prefix + ".bin";
            framedata_to_bin(frame, png_file);
            std::cout << "Saved " << png_file << std::endl;

            rs2_intrinsics intrinsics = vfsp.as<rs2::video_stream_profile>().get_intrinsics();
            rs2_extrinsics extrinsics = vfsp.get_extrinsics_to(vfsp);
        }
    }
}

void rs2wrapper::save_calib()
{
    std::string csv_file = path_map["Calib"] + "/calib.csv";
    std::ofstream csv;
    csv.open(csv_file);

    // Intrinsics of color & depth frames
    rs2::stream_profile profile_color = profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics intr_color = profile_color.as<rs2::video_stream_profile>().get_intrinsics();
    // Fetch stream profile for depth stream
    // Downcast to video_stream_profile and fetch intrinsics
    rs2::stream_profile profile_depth = profile.get_stream(RS2_STREAM_DEPTH);
    rs2_intrinsics intr_depth = profile_depth.as<rs2::video_stream_profile>().get_intrinsics();

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

    std::vector<rs2::sensor> sensors = profile.get_device().query_sensors();
    for (auto &&sensor : sensors)
    {
        if (auto dss = sensor.as<rs2::depth_stereo_sensor>())
        {
            csv << dss.get_depth_scale() << ","
                << dss.get_stereo_baseline() << ",\n";
        }
    }

    std::cout << "Save camera calibration data..." << std::endl;
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
