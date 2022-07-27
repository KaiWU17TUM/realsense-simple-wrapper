#include "utils.h"

void print(const std::string &msg, int mode)
{
    if (mode == 0)
        std::cout << "[INFO] : " << msg << std::endl;
    if (mode == 1)
        std::cout << "[WARN] : " << msg << std::endl;
    if (mode == 2)
        std::cout << "[ERRO] : " << msg << std::endl;
}

std::string pad_zeros(const std::string &in_str, const size_t &num_zeros)
{
    std::string out_str =
        std::string(num_zeros - std::min(num_zeros, in_str.length()), '0') +
        in_str;
    return out_str;
}

std::int64_t get_timestamp_duration_ns(const std::chrono::steady_clock::time_point &timestamp_start)
{
    auto timestamp_now = std::chrono::steady_clock::now();
    std::int64_t timestamp_diff =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            timestamp_now - timestamp_start)
            .count();
    return timestamp_diff;
}

argparser::argparser(int &argc, char **argv)
{
    for (int i = 1; i < argc; ++i)
        tokens.push_back(std::string(argv[i]));
}

std::string argparser::get(const std::string &option)
{
    std::vector<std::string>::const_iterator itr;
    itr = std::find(tokens.begin(), tokens.end(), option);
    if (itr != tokens.end() && ++itr != tokens.end())
    {
        return *itr;
    }
    static const std::string empty_string("");
    return empty_string;
}

bool argparser::check(const std::string &option)
{
    return std::find(tokens.begin(), tokens.end(), option) != tokens.end();
}
