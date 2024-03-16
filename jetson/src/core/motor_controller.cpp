#include "motor_controller.h"
#include "log.h"

#include <unistd.h>
#include <math.h>
#include <stdexcept>


MotorController::MotorController(double pterm, double iterm, double dterm, double integratorClamp, 
		double enc_2_dist, double idealSpeed, int mc, int motor, int encoderVal, int file) :
	ctrller(pterm, iterm, dterm, integratorClamp), idealSpeed(idealSpeed), enc_2_dist(enc_2_dist), mc(mc), motor(motor), lastEncoderVal(encoderVal), file(file){
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
	uint8_t data[4] = {mc, motor, WRITE_CMD, (uint8_t) saturatedMotorSignal};
	write_i2c(file, MCU_M, data, 4);
}

void MotorController::resetElapsedDistance() {
	elapsedDistance = 0;
}

double MotorController::getElapsedDistance() {
	return elapsedDistance;
}

// Stepper Motor Controller
// 1st byte: which stepper motor (X: 0, Y: 1, Z: 0)
// 2nd + 3rd byte: 16 bit position, mm location x 100
// Here we want to move keep moving forward (program will stop after hitting a limit switch)
void MotorController::simple_go(){
	uint8_t data[4] = {0};
	data[0] = motor;
	data[1] = 0xFF;
	data[2] = 0xFF;
	write_i2c(file, MCU_E, data, 4);
}

void MotorController::simple_bkwd(){
	uint8_t data[4] = {0};
	data[0] = motor;
	write_i2c(file, MCU_E, data, 4);
}

void MotorController::simple_pos(double pos){
	pos = pos * 1000; // convert from [m] to [mm]
	uint16_t sat_pos;
	if (pos > 6535){
		sat_pos = 0xFFFF;
	} else {
		sat_pos = (uint16_t) (pos*10);
	}
	uint8_t data[4] = {0};
	data[0] = motor;
	data[1] = (sat_pos & 0xFF00) >> 8;
	data[2] = (sat_pos & 0xFF);
	write_i2c(file, MCU_E, data, 4);
	log(	std::string("Move stepper motor: ") + std::to_string(motor) +
			std::string("to position: ") + std::to_string(pos) + 
			std::string("by sending value: ") + std::to_string(data[1]) 
			+ std::string(" ") + std::to_string(data[2]) );
	// uint8_t resp = 0;
	// while(resp == 0){
	//	read_i2c(file, MCU_E, &resp, 1);
	//	usleep(1000);
	// }
	// log(	std::string("Read stepper motor return: ") + std::to_string(resp));
	// if (resp == 2){ // hit limit switch
	// 	throw std::logic_error("hit limit switch");
	// }
}
