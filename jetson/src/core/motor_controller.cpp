#include "motor_controller.h"

#include <math.h>

MotorController::MotorController(double pterm, double iterm, double dterm, double integratorClamp, double radius, double idealSpeed) :
	ctrller(pterm, iterm, dterm, integratorClamp), idealSpeed(idealSpeed), radius(radius) {
	lastUpdateTime = std::chrono::system_clock::now();
	elapsedDistance = 0;
	lastEncoderVal = 0;
	// TODO: read encoder val
}

void MotorController::setIdealSpeed(double idealSpeed) {
	idealSpeed = idealSpeed;
}

void MotorController::update(uint16_t encoderVal) {	
	std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
	double deltaT = (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastUpdateTime)).count();
	lastUpdateTime = currentTime;

	// TODO: update based on pulses per rotation
	// TODO: need to deal with encoder overflow somehow
	double distance = (encoderVal-lastEncoderVal)*2*M_PI/8*2*M_PI*radius;
	elapsedDistance += distance;

	double currentSpeed =  distance / deltaT;
	lastEncoderVal = encoderVal;

	double motorSignal = ctrller.update_ctrl_signal(idealSpeed - currentSpeed, deltaT);
	
	// TODO: write motorSignal to mcu1
}

void MotorController::resetElapsedDistance() {
	elapsedDistance = 0;
}

double MotorController::getElapsedDistance() {
	return elapsedDistance;
}