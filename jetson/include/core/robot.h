#ifndef ROBOT_H
#define ROBOT_H

#include "motor_controller.h"
#include "imaging.h"
#include "point.h"

enum DriveMotor { frontLeft, backLeft, frontRight, backRight };

namespace Arm {
	enum ServoMotor { x, y, z };
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

	// Robot orientation: x y theta
	Point2D robotPosition;
	double robotAngle;
	// Arm position: x y z. Relative to robot frame
	Point3D armPosition;

	int i2c_bus_file;

public:


	Robot(double robotLength, double robotWidth, double wheelRadius, Camera& camera, double robotPosTol, double armPosTol);
	Point2D getRobotPosition();
	double getRobotAngle();
	void updateRobotOrientation();
	Point3D getArmPosition();
	void updateArmPosition();

	double calculate_wheel_speed(double v, double w);
	void driveRobotForward(Point2D idealPos);
	void resetServoArm(Arm::ServoMotor motor);
	void moveServoArm(Arm::ServoMotor motor, double pos);
	void pollinate();
	// TODO: add logging somewhere
};

#endif // h