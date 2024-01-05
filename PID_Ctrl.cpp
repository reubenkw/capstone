#include "PID_Ctrl.h"

#include <cassert>

PID_Ctrl::PID_Ctrl(double pterm, double iterm, double dterm, int intWindowSize, int deriveWindowSize) :
    pterm(pterm), iterm(iterm), dterm(dterm), intWindowSize(intWindowSize), deriveWindowSize(deriveWindowSize) { }

double PID_Ctrl::update_ctrl_signal(double error, double timestep){

    double derivative = (error-last_error)/timestep;
    // average derivative
    if (counter < deriveWindowSize){
        derivativehistory.push_back(derivative);
        derivativeSum += derivative;
    } else {
        derivativeSum -= derivativehistory.at(counter%deriveWindowSize);
        derivativehistory.at(counter%deriveWindowSize) = derivative;
        derivativeSum += derivative;
    }

    double error_inc = error * timestep;
    // moving window integral
    if (counter < intWindowSize){
        history.push_back(error_inc);
        integratorSum += error_inc;
    } else {
        integratorSum -= history.at(counter%intWindowSize);
        history.at(counter%intWindowSize) = error_inc;
        integratorSum += error_inc;
    }    
    
    counter++;

    return pterm * error 
        + dterm * derivativeSum/derivativehistory.size()
        + iterm * integratorSum;
    
}