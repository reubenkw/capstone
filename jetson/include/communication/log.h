#ifndef LOG_H
#define LOG_H

#include <string>

const bool DEBUG = true;

void initialize_log();
void log(std::string message);
void clear_log();
void debug_log(std::string message);
std::string getFormattedTimeStamp();

#endif //h 
