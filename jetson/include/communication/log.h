#ifndef LOG_H
#define LOG_H

#include <string>

const bool DEBUG = false;

void initialize_log();
void log(std::string message);
void clear_log();
std::string getFormattedTimeStamp();

#endif //h 