#include "robot.h"

#include <cmath>

#define IDEAL_LINEAR_SPEED 0.1

Robot::Robot(double robotLength, double robotWidth, double wheelRadius, Camera & camera, double robotPosTol, double armPosTol) 
    : robotLength(robotLength), robotWidth(robotWidth), wheelRadius(wheelRadius), camera(camera), robotPosTol(robotPosTol), armPosTol(armPosTol) {
    
	readEncoderVals();
	
	// TODO: pid terms need to be determined experimentally
    drive[frontLeft] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, frontLeft, encoderVal[frontLeft]);
	drive[backLeft] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, backLeft, encoderVal[backLeft]);
    drive[frontRight] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, frontRight, encoderVal[frontRight]);
    drive[backRight] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, backRight, encoderVal[backRight]);

	// 4 bc 4 drive motors
	servoArm[Arm::x] = MotorController(10, 1, 1, 5, 0, 0, SERVO_MC, Arm::x, encoderVal[4 + Arm::x]);
	servoArm[Arm::y] = MotorController(10, 1, 1, 5, 0, 0, SERVO_MC, Arm::y, encoderVal[4 + Arm::y]);
	servoArm[Arm::z] = MotorController(10, 1, 1, 5, 0, 0, SERVO_MC, Arm::z, encoderVal[4 + Arm::z]);

	robotPosition = Point(0, 0, 0);
	armPosition = Point(0, 0, 0);

	i2c_bus_file = open_i2c();
}

Point Robot::getRobotPosition() {
	return robotPosition;
}

Point Robot::getArmPosition() {
	return armPosition;
}

void Robot::readEncoderVals(){
	read_i2c(i2c_bus_file, MCU_ENCODER, (uint8_t * )encoderVal, 14);
}

// Based on MTE 544 Localization I and II 
void Robot::updateRobotPosition() {
	double d_left = (drive[frontLeft].getElapsedDistance() + drive[backLeft].getElapsedDistance()) / 2.0;
	double d_right = (drive[frontRight].getElapsedDistance() + drive[backRight].getElapsedDistance()) / 2.0;
	double d_avg = (d_left + d_right)/2;

	double rotation = (d_right - d_left)/robotWidth;

	robotPosition[roboPos::theta] += rotation;
	robotPosition[roboPos::x] += d_avg*cos(robotPosition[roboPos::theta]);
	robotPosition[roboPos::y] += d_avg*sin(robotPosition[roboPos::theta]);
}

void Robot::updateArmPosition() {
	armPosition[0] = servoArm[0].getElapsedDistance();
	armPosition[1] = servoArm[1].getElapsedDistance();
	armPosition[2] = servoArm[2].getElapsedDistance();
}

// From MTE 544 Control Lecture Slide 9
double calculate_radius(double delta_x, double delta_y) {
	double L = sqrt(delta_x * delta_x + delta_y * delta_y);

	return L * L / 2 * std::abs(delta_y);
}

// From MTE 544 Modeling III IV Lecture Slide 16
// Pass in -w for left wheel
// Assume positive w is CCW orientation
double Robot::calculate_wheel_speed(double v, double w) {
	return 1 / wheelRadius * (v + robotWidth * w / 2 + std::pow(robotLength * w / 2, 2) / (v + robotWidth * w / 2));
}

void Robot::driveRobotForward(Point delta) {
	robotPosition = Point(0, 0, 0);
	drive[frontLeft].resetElapsedDistance();
	drive[backLeft].resetElapsedDistance();
	drive[frontRight].resetElapsedDistance();
	drive[backRight].resetElapsedDistance();

	while (delta.mag() < robotPosTol) {

		double r = calculate_radius(delta.x, delta.y);

		// TODO: how to figure ideal v?
		double v = IDEAL_LINEAR_SPEED;
		double w = v / r;
		double leftWheelSpeed = calculate_wheel_speed(v, -1 * w);
		double rightWheelSpeed = calculate_wheel_speed(v, w);

		drive[frontLeft].setIdealSpeed(leftWheelSpeed);
		drive[backLeft].setIdealSpeed(leftWheelSpeed);
		drive[frontRight].setIdealSpeed(rightWheelSpeed);
		drive[backRight].setIdealSpeed(rightWheelSpeed);

		readEncoderVals();

		drive[frontLeft].update(i2c_bus_file, encoderVal[frontLeft]);
		drive[backLeft].update(i2c_bus_file, encoderVal[backLeft]);
		drive[frontRight].update(i2c_bus_file, encoderVal[frontRight]);
		drive[backRight].update(i2c_bus_file, encoderVal[backRight]);

		updateRobotPosition();

		delta = delta - robotPosition;
	}

	drive[frontLeft].setIdealSpeed(0);
	drive[backLeft].setIdealSpeed(0);
	drive[frontRight].setIdealSpeed(0);
	drive[backRight].setIdealSpeed(0);

}

void Robot::resetServoArm(Arm::ServoMotor motor) {
	bool limitSwitch = false;
	// TODO: read limit switch values servo motor
	// TODO: figure out direction
	double idealSpeed = IDEAL_LINEAR_SPEED;

	servoArm[motor].setIdealSpeed(idealSpeed);

	while (!limitSwitch) {
		// TODO: limitSwitch = read from limit switch 
	}
	servoArm[motor].setIdealSpeed(0);
	armPosition[motor] = 0;
	servoArm[motor].resetElapsedDistance();
}

void Robot::moveServoArm(Arm::ServoMotor motor, double pos) {
	double delta = pos - armPosition[pos];
	// TODO: how to figure ideal v?
	double idealSpeed = IDEAL_LINEAR_SPEED;

	// TODO: do we need PID controllers ideal speed of servo arm position? 
	servoArm[motor].setIdealSpeed(idealSpeed);

	while (delta < armPosTol) {
		readEncoderVals();

		// 4 bc 4 drive motors
		servoArm[motor].update(i2c_bus_file, encoderVal[4 + motor]);
		updateArmPosition();
		delta = pos - armPosition[motor];
	}
	servoArm[motor].setIdealSpeed(0);
}

// TODO: perform pollination pattern
void Robot::pollinate() { }


