#ifndef ROBOT_H
#define ROBOT_H

#include "imaging.h"
#include "point.h"
#include "comm.h"
#include "constants.h"

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

const float limit_fwd[3] = {CARTESIAN_X_MAX, CARTESIAN_Y_MAX, CARTESIAN_Z_MAX};
const float limit_bkwd[3] = {CARTESIAN_X_MIN, CARTESIAN_Y_MIN, CARTESIAN_Z_MIN};

class Robot {
	// there's only one camera, everything should affect the same camera
	Camera& camera;

	// Robot orientation: x y
	Point2D robotPosition;
	// Arm position: x y z. Relative to robot frame
	Point3D armPosition;

	int i2c_bus_file;
public:
	Robot(Camera& camera);
	Point2D getRobotPosition();
	Point3D getArmPosition();
	uint16_t getServoMotorEncoderVal(ServoMotor motor);
	uint16_t getDriveMotorEncoderVal(DriveMotor motor);

	void setArmPosition(uint8_t motor_num, double pos);
	void resetServoArm();
	void pollinate();
	std::vector<Point3D> scan();
	std::vector<Point3D> findFlowers();
	void pollinate_row(int n = 1);
	void driveForwards(uint8_t pwm_speed, float seconds);
	void driveBackwards(uint8_t pwm_speed, float seconds);
	void pollinate_all_in_zone(std::vector<Point3D> flowerCenters);
};

#endif // h