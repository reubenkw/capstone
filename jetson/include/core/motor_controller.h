#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pid_ctrl.h"
#include "comm.h"

#include <chrono>
#include <set>

class MotorController {
	PID_Ctrl ctrller;
	double idealSpeed;

	double elapsedDistance;
	int lastEncoderVal;
	std::chrono::time_point<std::chrono::system_clock>  lastUpdateTime;	
	double radius;
	int mc;
	int reg;

public:
	MotorController(double pterm = 0, double iterm = 0, double dterm = 0.01, double integratorClamp = 0, 
		double radius = 0.01, double idealSpeed = 0, int mc = 0, int reg = 0, int encoderVal = 0);
	void setIdealSpeed(double idealSpeed);
	void update(int file, uint16_t encoderVal);
	void resetElapsedDistance();
	double getElapsedDistance();
};

#endif //h