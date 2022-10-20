#include "rs_utils.hpp"

void print_rs2_device_infos(const rs2::device &device, const bool &verbose)
{
    if (!verbose)
        return;

    std::cout << std::string(80, '=') << std::endl;
    std::cout << ">>>>> RS2_CAMERA_INFO <<<<<" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    std::cout << "Name          : " << device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << "Serial Number : " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    try
    {
        std::cout << "Product Line  : " << device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) << std::endl;
    }
    catch (const rs2::error &e)
    {
        std::cout << "not available, " << e.what() << std::endl;
    }
    try
    {
        std::cout << "Firmware      : " << device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    }
    catch (const rs2::error &e)
    {
        std::cout << "not available, " << e.what() << std::endl;
    }
    try
    {
        std::cout << "USB type      : " << device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
    }
    catch (const rs2::error &e)
    {
        std::cout << "not available, " << e.what() << std::endl;
    }
    std::cout << std::string(80, '=') << std::endl;
}

void print_camera_temperature(device &device,
                              const int &printout_interval,
                              const bool &verbose)
{
    if (!verbose)
        return;

    int c = device.camera_temp_printout_counter;
    if (c >= printout_interval || c == -1)
    {
        device.camera_temp_printout_counter = 0;
        auto dss = device.depth_sensor;
        if (dss->supports(RS2_OPTION_ASIC_TEMPERATURE))
        {
            auto temp = dss->get_option(RS2_OPTION_ASIC_TEMPERATURE);
            print(device.sn + " Temperature ASIC      : " + std::to_string(temp), 0);
        }
        if (dss->supports(RS2_OPTION_PROJECTOR_TEMPERATURE))
        {
            auto temp = dss->get_option(RS2_OPTION_PROJECTOR_TEMPERATURE);
            print(device.sn + " Temperature Projector : " + std::to_string(temp), 0);
        }
    }
    device.camera_temp_printout_counter += 1;
}

void print_no_device_enabled(const std::string &function)
{
    print("no device enabled, skipping '" + function + "' ...", 1);
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

void timestamp_to_txt(const int64_t &global_timestamp,
                      const rs2_metadata_type &color_timestamp,
                      const rs2_metadata_type &depth_timestamp,
                      const std::string &filename)
{
    std::fstream txt;
    txt.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
    // If file does not exist, Create new file
    if (!txt)
        txt.open(filename, std::fstream::in | std::fstream::out | std::fstream::trunc);
    txt << global_timestamp
        << "::"
        << color_timestamp
        << "::"
        << depth_timestamp
        << "\n";
    txt.close();
}

bool check_if_color_depth_frames_are_valid(const rs2::frameset &frameset)
{
    rs2::frame cf = frameset.first_or_default(RS2_STREAM_COLOR);
    rs2::frame df = frameset.first_or_default(RS2_STREAM_DEPTH);
    if (!cf || !df)
        return false;
    else
        return true;
}

// [STORAGEPATH CLASS] ---------------------------------------------------------
storagepath::storagepath()
{
}

void storagepath::create(const std::vector<std::string> &device_sns,
                         const std::string &base_path)
{
    time(&trial_idx);
    for (auto const &device_sn : device_sns)
    {
        this->create(device_sn, base_path);
    }
}

void storagepath::create(const std::string &device_sn,
                         const std::string &base_path)
{
    this->device_sns.push_back(device_sn);
    // Base
    std::string path(base_path);
    this->make_dirs(path.c_str(), true);
    // Device
    path.append("/");
    path.append(device_sn);
    this->make_dirs(path.c_str(), true);
    // System time , trial
    path.append("/");
    path.append(std::to_string(trial_idx));
    this->make_dirs(path.c_str(), false);
    // Timestamp
    this->timestamp[device_sn] = path + "/timestamp";
    this->make_dirs(this->timestamp[device_sn].c_str(), false);
    // Calib
    this->calib[device_sn] = path + "/calib";
    this->make_dirs(this->calib[device_sn].c_str(), false);
    // Color
    this->color[device_sn] = path + "/color";
    this->make_dirs(this->color[device_sn].c_str(), false);
    this->color_metadata[device_sn] = path + "/color_metadata";
    this->make_dirs(this->color_metadata[device_sn].c_str(), false);
    // Depth
    this->depth[device_sn] = path + "/depth";
    this->make_dirs(this->depth[device_sn].c_str(), false);
    this->depth_metadata[device_sn] = path + "/depth_metadata";
    this->make_dirs(this->depth_metadata[device_sn].c_str(), false);
}

void storagepath::show()
{
    for (auto const &device_sn : device_sns)
        show(device_sn);
}

void storagepath::show(const std::string &device_sn)
{
    print("calib : " + calib[device_sn], 0);
    print("color : " + color[device_sn], 0);
    print("depth : " + depth[device_sn], 0);
    print("color_metadata : " + color_metadata[device_sn], 0);
    print("depth_metadata : " + depth_metadata[device_sn], 0);
}

int storagepath::make_dirs(const char *path, const bool &exists_ok)
{
    int status = 0;
    errno = 0;
    status = mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status < 0)
    {
        if (errno == EEXIST)
        {
            if (!exists_ok)
            {
                print("Folder exists : " + std::string(path), 1);
            }
        }
        else
        {
            print(std::string(std::strerror(errno)) + " : " + std::string(path), 2);
        }
    }
    return status;
}
// --------------------------------------------------------- [STORAGEPATH CLASS]