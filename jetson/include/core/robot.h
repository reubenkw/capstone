#ifndef ROBOT_H
#define ROBOT_H

#include "motor_controller.h"
#include "imaging.h"
#include "point.h"
#include "comm.h"

class Robot {
	double robotLength;
	double robotWidth;
	double wheelRadius;

	double robotPosTol;
	double armPosTol;

	MotorController drive[4];
	MotorController servoArm[3];

	// there's only one camera, everything should affect the same camera
	Camera& camera;

	// Robot position: x y theta
	Point robotPosition;
	// Arm position: x y z
	Point armPosition;

	int i2c_bus_file;

	uint16_t encoderVal[7];

public:


	Robot(double robotLength, double robotWidth, double wheelRadius, Camera& camera, double robotPosTol, double armPosTol);
	Point getRobotPosition();
	void updateRobotPosition();
	Point getArmPosition();
	void updateArmPosition();

	double calculate_wheel_speed(double v, double w);
	void driveRobotForward(Point idealPos);
	void resetServoArm(Arm::ServoMotor motor);
	void moveServoArm(Arm::ServoMotor motor, double pos);
	void pollinate();
	void readEncoderVals();
	// TODO: add logging somewhere
};

#endif // h