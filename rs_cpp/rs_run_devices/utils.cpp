#include "utils.hpp"

void print(const std::string &msg, const int &mode)
{
    if (mode == 0)
        std::cout << "[INFO] : " << msg << std::endl;
    if (mode == 1)
        std::cout << "[WARN] : " << msg << std::endl;
    if (mode == 2)
        std::cout << "[ERRO] : " << msg << std::endl;
}

std::string pad_zeros(const std::string &in_str, const int &num_zeros)
{
    std::string out_str = "";
    out_str += std::string(num_zeros - std::min(num_zeros, (int)in_str.length()), '0');
    out_str += in_str;
    return out_str;
}

int64_t get_timestamp_duration_ns(const std::chrono::steady_clock::time_point &timestamp_start)
{
    auto timestamp_now = std::chrono::steady_clock::now();
    int64_t timestamp_diff =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            timestamp_now - timestamp_start)
            .count();
    return timestamp_diff;
}

argparser::argparser()
{
}

argparser::argparser(int argc, char *argv[])
{
    for (int i = 1; i < argc; ++i)
        args.push_back(std::string(argv[i]));
}

argparser::~argparser()
{
}

std::string argparser::getarg(const std::string &option)
{
    std::vector<std::string>::const_iterator itr;
    itr = std::find(args.begin(), args.end(), option);
    if (itr != args.end() && ++itr != args.end())
    {
        return *itr;
    }
    static const std::string empty_string("");
    return empty_string;
}

bool argparser::checkarg(const std::string &option)
{
    return std::find(args.begin(), args.end(), option) != args.end();
}

void argparser::printout()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << ">>>>> rs2args <<<<<" << std::endl;
    std::cout << "========================================" << std::endl;
    for (int i = 0; i < args.size(); i += 2)
        print(args[i] + "  " + args[i + 1], 1);
    std::cout << "========================================\n"
              << std::endl;
}