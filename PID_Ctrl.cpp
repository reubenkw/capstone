#include "PID_Ctrl.h"

#include <cassert>

PID_Ctrl::PID_Ctrl(double pterm, double iterm, double dterm, double integratorClamp) :
    pterm(pterm), iterm(iterm), dterm(dterm), integratorClamp(integratorClamp) {
        assert(("integrator clamp value should be positive", integratorClamp >= 0 ));
     }

double PID_Ctrl::update_ctrl_signal(double error, double timestep){

    double derivative = (error-last_error)/timestep;
    double error_inc = error * timestep;

    integratorSum += error_inc;

    if (integratorSum > integratorClamp) {
        integratorSum = integratorClamp;
    } else if (integratorSum < -1 * integratorClamp) {
        integratorSum = -1 * integratorClamp;
    }
    
    return pterm * error 
        + dterm * derivative
        + iterm * integratorSum;
    
}