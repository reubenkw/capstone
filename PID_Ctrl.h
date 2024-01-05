#ifndef PID_H
#define PID_H

#include <vector>

class PID_Ctrl{
    double pterm;
    double iterm;
    double dterm;
    
    int counter = 0; 
    std::vector<double> history;
    std::vector<double> derivativehistory;
    double integratorSum = 0;
    double derivativeSum = 0;
    double last_error = 0;

    int intWindowSize;
    int deriveWindowSize;

    public:
        PID_Ctrl(double pterm, double iterm, double dterm, int intWindowSize = 100, int deriveWindowSize = 5);
        double update_ctrl_signal(double error, double timestep);
};

#endif //h