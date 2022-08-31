#include "rs_utils.hpp"

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
    time(&trial_idx);
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
    path.append("/");
    path.append(std::to_string(trial_idx));
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
