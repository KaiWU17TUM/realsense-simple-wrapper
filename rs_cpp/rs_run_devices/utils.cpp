#include "utils.hpp"

void print(const std::string &msg, const int &mode)
{
    if (mode == 0)
        std::cout << "[INFO] : " << msg << std::endl;
    if (mode == 1)
        std::cout << "[WARN] : " << msg << std::endl;
    if (mode == 2)
        std::cerr << "[ERRO] : " << msg << std::endl;
}

std::string pad_zeros(const std::string &in_str, const int &num_zeros)
{
    int string_len = num_zeros - std::min(num_zeros, (int)in_str.length());
    std::string out_str = "";
    out_str += std::string(string_len, '0');
    out_str += in_str;
    return out_str;
}

int64_t get_timestamp_ns()
{
    int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count();
    return ns;
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

int argparser::getargi(const std::string &option)
{
    return std::stoi(getarg(option));
}

bool argparser::getargb(const std::string &option)
{
    bool o;
    std::istringstream(getarg(option)) >> std::boolalpha >> o;
    return o;
}

bool argparser::checkarg(const std::string &option)
{
    return std::find(args.begin(), args.end(), option) != args.end();
}

void argparser::printout()
{
    std::cout << std::string(80, '=') << std::endl;
    std::cout << ">>>>> rs2args <<<<<" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    for (int i = 0; i < args.size(); i += 2)
        std::cout << args[i] + " : " + args[i + 1] << std::endl;
    std::cout << std::string(80, '=') << std::endl;
}