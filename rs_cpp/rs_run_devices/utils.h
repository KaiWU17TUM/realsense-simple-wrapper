#include <iostream>

void print(const std::string &str_in, int mode = 0)
{
    if (mode == 0)
        std::cout << "[INFO] : " << str_in << std::endl;
    if (mode == 1)
        std::cout << "[WARN] : " << str_in << std::endl;
    if (mode == 2)
        std::cout << "[ERRO] : " << str_in << std::endl;
}