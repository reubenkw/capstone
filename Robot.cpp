#include "robot.h"

#include <cmath>

#define IDEAL_LINEAR_SPEED 0.1

Robot::Robot(double robotLength, double robotWidth, double wheelRadius, Camera & camera, double robotPosTol, double armPosTol) 
    : robotLength(robotLength), robotWidth(robotWidth), wheelRadius(wheelRadius), camera(camera), robotPosTol(robotPosTol), armPosTol(armPosTol) {
    // TODO: pid terms need to be determined experimentally
    drive[frontLeft] = MotorController(10, 1, 1, 5, 0);
    drive[backLeft] = MotorController(10, 1, 1, 5, 0);
    drive[frontRight] = MotorController(10, 1, 1, 5, 0);
    drive[backRight] = MotorController(10, 1, 1, 5, 0);

	servoArm[x] = MotorController(10, 1, 1, 5, 0);
	servoArm[y] = MotorController(10, 1, 1, 5, 0);
	servoArm[z] = MotorController(10, 1, 1, 5, 0);

	robotPosition = Point(0, 0, 0);
	armPosition = Point(0, 0, 0);
}

Point Robot::getRobotPosition() {
	return robotPosition;
}

Point Robot::getArmPosition() {
	return armPosition;
}

// how are we going to do this? 
// should it just be called at the end of each robot move function? 
void Robot::updateRobotPosition() {}
void Robot::updateArmPosition() {}

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

void Robot::driveRobotForward(Point idealPos) {
	Point delta = idealPos - robotPosition;

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

		updateRobotPosition();

		delta = idealPos - robotPosition;
	}

}

void Robot::moveServoArm(ServoMotor motor, double pos) {
	double delta = pos - armPosition[motor];
	while (delta < armPosTol) {
		// TODO: how to figure ideal v?
		double idealSpeed = IDEAL_LINEAR_SPEED;

		// TODO: do we need PID controllers ideal speed of servo arm position? 
		servoArm[motor].setIdealSpeed(idealSpeed);
		updateArmPosition();
		delta = pos - armPosition[motor];
	}
}

// TODO: perform pollination pattern
void Robot::pollinate() { }


