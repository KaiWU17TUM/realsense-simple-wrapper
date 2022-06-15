#include "rs_wrapper.h"

int num_zeros_to_pad = NUM_ZEROS_TO_PAD;

rs2wrapper::rs2wrapper(int argc,
                       char *argv[],
                       rs2::context ctx) : rs2args(argc, argv)
{
    // prints out info
    info();
    // Create save directory
    mkdir(save_path(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // Add network device context
    if (network())
    {
        rs2::net_device dev(ip());
        std::cout << "IP address found..." << std::endl;
        dev.add_to(ctx);
    }
    // Configure pipeline config
    // Start pipeline
    pipe = rs2::pipeline(ctx);
    cfg.enable_stream(RS2_STREAM_COLOR, width(), height(), RS2_FORMAT_RGB8, fps());
    cfg.enable_stream(RS2_STREAM_DEPTH, width(), height(), RS2_FORMAT_Z16, fps());
    rs2::pipeline_profile profile = pipe.start(cfg);
    std::cout << "Pipeline started..." << std::endl;
    std::cout << "Initialized realsense device..." << std::endl;
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
        if (rs2::video_frame vf = frame.as<rs2::video_frame>())
        {
            std::string prefix = filename_prefix_with_timestamp(
                vf, save_file_prefix, num_zeros_to_pad);

            // Record per-frame metadata for UVC streams
            std::string csv_file = (std::string)save_path() + "/" +
                                   prefix +
                                   vf.get_profile().stream_name() +
                                   "-metadata.csv";
            metadata_to_csv(vf, csv_file);

            // Write images to disk
            std::string png_file = (std::string)save_path() + "/" +
                                   prefix +
                                   vf.get_profile().stream_name() +
                                   ".bin";
            framedata_to_bin(frame, png_file);
            std::cout << "Saved " << png_file << std::endl;
        }
    }
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
