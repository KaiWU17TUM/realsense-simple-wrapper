#include <iostream>
#include <chrono>
#include <thread>

/**
 * @brief Prints out a message using cout with either INFO/WARN/ERR tags.
 *
 * @param msg message to be printed
 * @param mode which tags to use
 */
void print(const std::string &msg, int mode = 0);

/**
 * @brief pads a string with zeros.
 *
 * https://stackoverflow.com/questions/6143824/add-leading-zeros-to-string-without-sprintf
 *
 * @param in_str input string
 * @param num_zeros number of zeros to pad
 * @return std::string
 */
std::string pad_zeros(const std::string &in_str, const size_t &num_zeros);

std::int64_t get_timestamp_duration_ns(const std::chrono::steady_clock::time_point &timestamp_start);
