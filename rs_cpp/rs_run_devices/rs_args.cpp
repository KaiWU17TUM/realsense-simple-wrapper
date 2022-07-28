#include "rs_args.hpp"

rs2args::rs2args(int argc, char *argv[]) : argparser(argc, argv)
{
}

int rs2args::steps()
{
    return std::stoi(getarg("--steps"));
}

int rs2args::fps()
{
    return std::stoi(getarg("--fps"));
}

int rs2args::height()
{
    return std::stoi(getarg("--height"));
}

int rs2args::width()
{
    return std::stoi(getarg("--width"));
}

rs2_format rs2args::color_format()
{
    if (_SUPPORTED_FORMATS.count(getarg("--color-format")))
        return _SUPPORTED_FORMATS[getarg("--color-format")];
    else
        throw std::invalid_argument("rs color format unknown");
}

rs2_format rs2args::depth_format()
{
    if (_SUPPORTED_FORMATS.count(getarg("--depth-format")))
        return _SUPPORTED_FORMATS[getarg("--depth-format")];
    else
        throw std::invalid_argument("rs depth format unknown");
}

std::string rs2args::save_path()
{
    return getarg("--save-path");
}

std::string rs2args::ip()
{
    return getarg("--ip");
}

bool rs2args::network()
{
    if (checkarg("--ip"))
        return true;
    else
        return false;
}

void rs2args::print_args()
{
    printout();
}