#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pid_ctrl.h"
#include <chrono>

class MotorController {
	PID_Ctrl ctrller;
	double idealSpeed;

	std::chrono::time_point<std::chrono::system_clock>  lastUpdateTime;

	double readEncoderValue();
	void outputMotorVoltage();

public:
	MotorController(double pterm = 0, double iterm = 0, double dterm = 0, double integratorClamp = 0, double idealSpeed = 0);
	void setIdealSpeed(double idealSpeed);

};

#endif //h