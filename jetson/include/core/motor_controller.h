#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pid_ctrl.h"

#include <chrono>
#include <set>

class MotorController {
	PID_Ctrl ctrller;
	double idealSpeed;

	double elapsedDistance;
	int lastEncoderVal;
	std::chrono::time_point<std::chrono::system_clock>  lastUpdateTime;	
	double radius;

public:
	MotorController(double pterm = 0, double iterm = 0, double dterm = 0, double integratorClamp = 0, double radius = 0.01, double idealSpeed = 0);
	void setIdealSpeed(double idealSpeed);
	void update(uint16_t encoderVal);
	void resetElapsedDistance();
	double getElapsedDistance();
};

#endif //h