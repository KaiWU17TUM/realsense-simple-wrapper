#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>

bool stob(const std::string &x);

/**
 * @brief Prints out a message using cout with either INFO/WARN/ERR tags.
 *
 * @param msg message to be printed
 * @param mode which tags to use
 */
void print(const std::string &msg, const int &mode);

/**
 * @brief pads a string with zeros.
 *
 * https://stackoverflow.com/questions/6143824/add-leading-zeros-to-string-without-sprintf
 *
 * @param in_str input string
 * @param num_zeros number of zeros to pad
 * @return std::string
 */
std::string pad_zeros(const std::string &in_str, const int &num_zeros);

/**
 * @brief Get the timestamp/duration in ns
 *
 * @param timestamp_start chrono steady clock timepoint.
 * @return int64_t UNIX timestamp in ns using system clock.
 */
int64_t get_timestamp_ns();
int64_t get_timestamp_duration_ns(const std::chrono::steady_clock::time_point &timestamp_start);

/**
 * @brief parses in put arguments.
 *
 * Taken from: https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
 *
 */
class argparser
{
    std::vector<std::string> args;

public:
    argparser();
    argparser(int argc, char **argv);
    ~argparser();
    std::string getarg(const std::string &option);
    int getargi(const std::string &option);
    bool getargb(const std::string &option);
    bool checkarg(const std::string &option);
    void printout();
};

#endif