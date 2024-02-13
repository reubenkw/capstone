#ifndef ROBOT_H
#define ROBOT_H

#include "motor_controller.h"
#include "imaging.h"
#include "point.h"
#include "comm.h"

enum DriveMotor { frontLeft, backLeft, frontRight, backRight };

enum ServoMotor { x, y, z };

class Robot {
	double robotLength;
	double robotWidth;
	double cameraHeight;

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

	uint16_t encoderVal[7];

public:


	Robot(double robotLength, double robotWidth, double cameraHeight, double wheelRadius, Camera& camera, double robotPosTol, double armPosTol);
	Point2D getRobotPosition();
	double getRobotAngle();
	void updateRobotOrientation();
	Point3D getArmPosition();
	void updateArmPosition();

	double calculate_wheel_speed(double v, double w);
	void driveRobotForward(Point2D idealPos);
	void resetServoArm(ServoMotor motor);
	void moveServoArm(ServoMotor motor, double pos);
	void pollinate();
	std::vector<Point3D> scan();
	void readEncoderVals();
	// TODO: add logging somewhere
};

#endif // h