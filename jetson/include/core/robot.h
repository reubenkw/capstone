#ifndef ROBOT_H
#define ROBOT_H

#include "motor_controller.h"
#include "imaging.h"
#include "point.h"
#include "comm.h"

#include <optional>

enum DriveMotor { frontLeft, backLeft, frontRight, backRight };

enum ServoMotor { x, y, z };

// matches convention set by MCU1
enum LimitSwitch {
	x_min,
	x_max,
	y_min,
	y_max,
	z_max
};

class Robot {
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


	Robot(Camera& camera);
	Point2D getRobotPosition();
	double getRobotAngle();
	void updateRobotOrientation();
	Point3D getArmPosition();
	void updateArmPosition();
	uint16_t getServoMotorEncoderVal(ServoMotor motor);
	uint16_t getDriveMotorEncoderVal(DriveMotor motor);

	std::optional<LimitSwitch> determineLimitSwitch(ServoMotor m, bool positive);
	double calculate_wheel_speed(double v, double w);
	void driveRobotForward(Point2D idealPos);
	void moveServoArm(ServoMotor motor, double pos);
	void resetServoArm();
	void pollinate();
	std::vector<Point3D> scan();
	std::vector<Point3D> findFlowers();
	void readEncoderVals();
	void pollinate_row(int n = 1);
};

#endif // h