#include "log.h"

#include <fstream>
#include <iostream>

void log(std::string message) {
    std::ofstream log_file("log.txt", std::ios::app);
    log_file << message << std::endl;
    log_file.close();
}