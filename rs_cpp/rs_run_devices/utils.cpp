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