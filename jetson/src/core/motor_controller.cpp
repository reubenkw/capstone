#include "motor_controller.h"

#include <math.h>

MotorController::MotorController(double pterm, double iterm, double dterm, double integratorClamp, 
		double enc_2_dist, double idealSpeed, int mc, int reg, int encoderVal, int file) :
	ctrller(pterm, iterm, dterm, integratorClamp), idealSpeed(idealSpeed), enc_2_dist(enc_2_dist), mc(mc), reg(reg), lastEncoderVal(encoderVal), file(file){
	lastUpdateTime = std::chrono::system_clock::now();
	elapsedDistance = 0;
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
	double distance = (encoderVal-lastEncoderVal)*enc_2_dist;
	elapsedDistance += distance;

	double currentSpeed =  distance / deltaT;
	lastEncoderVal = encoderVal;

	double motorSignal = ctrller.update_ctrl_signal(idealSpeed - currentSpeed, deltaT);
	int8_t saturatedMotorSignal;
	if (motorSignal > 127){
		saturatedMotorSignal = 127;
	} else if (motorSignal < -128){
		saturatedMotorSignal = -128;
	} else {
		saturatedMotorSignal = motorSignal;
	}
	uint8_t data[4] = {mc, reg, WRITE_CMD, (uint8_t) saturatedMotorSignal};
	write_i2c(file, MCU_1, data, 4);
}

void MotorController::resetElapsedDistance() {
	elapsedDistance = 0;
}

double MotorController::getElapsedDistance() {
	return elapsedDistance;
}
