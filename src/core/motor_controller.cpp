#include "../core/motor_controller.h"


MotorController::MotorController(double pterm, double iterm, double dterm, double integratorClamp, double idealSpeed) :
	ctrller(pterm, iterm, dterm, integratorClamp), idealSpeed(idealSpeed) {
	lastUpdateTime = std::chrono::system_clock::now();
}

void MotorController::setIdealSpeed(double idealSpeed) {
	idealSpeed = idealSpeed;
}

// TODO: read from GPIO or something?
double MotorController::readEncoderValue() {
	return 1;
}

void MotorController::update() {
	double encoderVal = readEncoderValue();
	std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
	double deltaT = (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastUpdateTime)).count();
	lastUpdateTime = currentTime;

	double currentSpeed = encoderVal / deltaT;
	ctrller.update_ctrl_signal(idealSpeed - currentSpeed, deltaT);
}