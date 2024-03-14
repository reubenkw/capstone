#include "log.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>

std::string file_name;
void initialize_log(){
    file_name = std::string("log.txt");
    clear_log();
}

void log(std::string message) {
    std::ofstream log_file(file_name, std::ios::app);
    log_file << getFormattedTimeStamp() << " " << message << std::endl;
    log_file.close();
}

void clear_log(){
    std::ofstream log_file(file_name, std::ios::trunc);
    log_file.close();
}

std::string getFormattedTimeStamp(){
    const auto now = std::chrono::system_clock::now();
    const std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&t_c);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");

    return oss.str();
}

void debug_log(std::string message) {
    if (DEBUG){
        std::ofstream log_file(file_name, std::ios::app);
        log_file << getFormattedTimeStamp() << " " << message << std::endl;
        log_file.close();
    }
}