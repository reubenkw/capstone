#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pid_ctrl.h"
#include "comm.h"

#include <chrono>
#include <set>

#define STOP 0
#define GO 1
#define BKWD 2

class MotorController {
	PID_Ctrl ctrller;
	double idealSpeed;

	double elapsedDistance;
	int lastEncoderVal;
	std::chrono::time_point<std::chrono::system_clock>  lastUpdateTime;	
	double enc_2_dist;
	uint8_t mc;
	uint8_t motor;
	int file;

public:
	MotorController(double pterm = 0, double iterm = 0, double dterm = 0.01, double integratorClamp = 0, 
		double enc_2_dist = 0.01, double idealSpeed = 0, int mc = 0, int motor = 0, int encoderVal = 0, int file = 0);
	void setIdealSpeed(double idealSpeed);
	void update(uint16_t encoderVal);
	void resetElapsedDistance();
	double getElapsedDistance();
	bool simple_pos(double pos); 
};

#endif //h