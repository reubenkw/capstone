#include "motor_controller.h"

#include <math.h>

MotorController::MotorController(double pterm, double iterm, double dterm, double integratorClamp, 
		double radius, double idealSpeed, int mc, int reg, int encoderVal) :
	ctrller(pterm, iterm, dterm, integratorClamp), idealSpeed(idealSpeed), radius(radius), mc(mc), reg(reg), lastEncoderVal(encoderVal){
	lastUpdateTime = std::chrono::system_clock::now();
	elapsedDistance = 0;
}

void MotorController::setIdealSpeed(double idealSpeed) {
	idealSpeed = idealSpeed;
}

void MotorController::update(int file, uint16_t encoderVal) {	
	std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
	double deltaT = (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastUpdateTime)).count();
	lastUpdateTime = currentTime;

	// TODO: update based on pulses per rotation
	// TODO: need to deal with encoder overflow somehow
	double distance = (encoderVal-lastEncoderVal)*2*M_PI/8*2*M_PI*radius;
	elapsedDistance += distance;

	double currentSpeed =  distance / deltaT;
	lastEncoderVal = encoderVal;

	int motorSignal = std::max(ctrller.update_ctrl_signal(idealSpeed - currentSpeed, deltaT), 1.0)* 128 + 128;
	uint16_t data = mc << 12 | reg << 8 | motorSignal;
	write_i2c(file, MCU_1, data);

}

void MotorController::resetElapsedDistance() {
	elapsedDistance = 0;
}

double MotorController::getElapsedDistance() {
	return elapsedDistance;
}