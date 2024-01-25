#ifndef ROBOT_H
#define ROBOT_H

#include "motor_controller.h"
#include "imaging.h"
#include "point.h"

enum DriveMotor { frontLeft, backLeft, frontRight, backRight };
enum ServoMotor { x, y, z };

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

	Point robotPosition;
	Point armPosition;

	int i2c_bus_file;

public:


	Robot(double robotLength, double robotWidth, double wheelRadius, Camera& camera, double robotPosTol, double armPosTol);
	Point getRobotPosition();
	void updateRobotPosition();
	Point getArmPosition();
	void updateArmPosition();

	double calculate_wheel_speed(double v, double w);
	void driveRobotForward(Point idealPos);
	void moveServoArm(ServoMotor motor, double pos);
	void pollinate();
	// TODO: add logging somewhere
};

#endif // h