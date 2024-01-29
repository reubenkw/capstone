#ifndef ROBOT_H
#define ROBOT_H

#include "motor_controller.h"
#include "imaging.h"
#include "point.h"

enum DriveMotor { frontLeft, backLeft, frontRight, backRight };

namespace Arm {
	enum ServoMotor { x, y, z };
};
namespace roboPos {
	enum robotPosition { x, y, theta};
};

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
	// TODO: add logging somewhere
};

#endif // h