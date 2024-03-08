#include "robot.h"

#include "log.h"
#include "cluster.h"
#include "constants.h"
#include "frame_transform.h"

#include <cmath>
#include <unistd.h> // linux sleep
#include <optional>

Robot::Robot(Camera & camera) 
    : camera(camera){
    
	initialize_log();
	resetEncoderVals();
	i2c_bus_file = open_i2c();
	
	// TODO: pid terms need to be determined experimentally
	// TODO: not possitive about the DRIVE_ENC_2_DIST signs so test
    drive[frontLeft] 	= MotorController(10, 1, 1, 5, 0,  DRIVE_ENC_2_DIST, DRIVE_MC, frontLeft, 	encoderVal[frontLeft], 	i2c_bus_file);
	drive[backLeft] 	= MotorController(10, 1, 1, 5, 0,  DRIVE_ENC_2_DIST, DRIVE_MC, backLeft, 	encoderVal[backLeft],	i2c_bus_file);
    drive[frontRight]	= MotorController(10, 1, 1, 5, 0, -DRIVE_ENC_2_DIST, DRIVE_MC, frontRight, 	encoderVal[frontRight], i2c_bus_file);
    drive[backRight] 	= MotorController(10, 1, 1, 5, 0, -DRIVE_ENC_2_DIST, DRIVE_MC, backRight, 	encoderVal[backRight], 	i2c_bus_file);

	// 4 bc 4 drive motors
	servoArm[x] = MotorController(10, 1, 1, 5, 0, XY_ENC_2_DIST, SERVO_MC, x, encoderVal[4 + x], i2c_bus_file);
	servoArm[y] = MotorController(10, 1, 1, 5, 0, XY_ENC_2_DIST, SERVO_MC, y, encoderVal[4 + y], i2c_bus_file);
	servoArm[z] = MotorController(10, 1, 1, 5, 0, XY_ENC_2_DIST, SERVO_MC, z, encoderVal[4 + z], i2c_bus_file);

	robotPosition = Point2D(0, 0);
	robotAngle = 0;
	// arm starts in upright position
	armPosition = Point3D(0, 0, CARTESIAN_Z_MAX);
	debug_log("Initialized Robot");
}

Point2D Robot::getRobotPosition() {
	debug_log(std::string("Robot Position - x:") 
	+ std::to_string(robotPosition.x) 
	+ std::string(", y: ")
	+ std::to_string(robotPosition.y));
	return robotPosition;
}

double Robot::getRobotAngle() {
	debug_log(std::string("Robot Angle: ") + std::to_string(robotAngle));
	return robotAngle;
}

Point3D Robot::getArmPosition() {
	debug_log(std::string("Arm Position - x:") 
	+ std::to_string(armPosition.x) 
	+ std::string(", y: ")
	+ std::to_string(armPosition.y)
	+ std::string(", z: ")
	+ std::to_string(armPosition.z));
	return armPosition;
}

void Robot::resetEncoderVals(){
	uint8_t clear_command = 1;
	write_i2c(i2c_bus_file, MCU_ENCODER, &clear_command, 1);
}

void Robot::readEncoderVals(){
	read_i2c(i2c_bus_file, MCU_ENCODER, (uint8_t * )encoderVal, 14);
	std::string encoderString = std::string("");
	for (int i = 0; i < 7; i++){
		encoderString = std::to_string(encoderVal[i]) + std::string(" ");
	}
	debug_log(std::string("Encoders: ") + encoderString);
	
}

void Robot::readLimitVals(){
	read_i2c(i2c_bus_file, MCU_1, &limitVals, 1);
	std::string limitString = std::string("");
	for (int i = 0; i < 7; i++){
		limitString = std::to_string((limitVals & (1<<i)) >> i) + std::string(" ");
	}
	debug_log(std::string("Limit: ") + limitString);
}

// Based on MTE 544 Localization I and II 
void Robot::updateRobotOrientation() {
	double d_left = (drive[frontLeft].getElapsedDistance() + drive[backLeft].getElapsedDistance()) / 2.0;
	double d_right = (drive[frontRight].getElapsedDistance() + drive[backRight].getElapsedDistance()) / 2.0;
	double d_avg = (d_left + d_right)/2;

	double rotation = (d_right - d_left)/CARTESIAN_X_MAX;

	robotAngle += rotation;
	robotPosition.x += d_avg*cos(robotAngle);
	robotPosition.y += d_avg*sin(robotAngle);

	debug_log(std::string("Updated Robot Position - x:") 
	+ std::to_string(robotPosition.x) 
	+ std::string(", y: ")
	+ std::to_string(robotPosition.y)
	+ std::string(", angle: ")
	+ std::to_string(robotAngle));
}

void Robot::updateArmPosition() {
	armPosition[0] += servoArm[0].getElapsedDistance();
	armPosition[1] += servoArm[1].getElapsedDistance();
	armPosition[2] += servoArm[2].getElapsedDistance();
	debug_log(std::string("Updated Arm Position - x:") 
	+ std::to_string(armPosition.x) 
	+ std::string(", y: ")
	+ std::to_string(armPosition.y)
	+ std::string(", z: ")
	+ std::to_string(armPosition.z));
}

// From MTE 544 Modeling III IV Lecture Slide 16
// Pass in -w for left wheel
// Assume positive w is CCW orientation
double Robot::calculate_wheel_speed(double v, double w) {
	return 1 / WHEEL_RADIUS * (v + CARTESIAN_X_MAX * w / 2 + std::pow(CARTESIAN_Y_MAX * w / 2, 2) / (v + CARTESIAN_X_MAX * w / 2));
}

void Robot::driveRobotForward(Point2D goal) {
	double angular_error_gain = 2;
	robotPosition = Point2D(0, 0);
	resetEncoderVals();
	drive[frontLeft].resetElapsedDistance();
	drive[backLeft].resetElapsedDistance();
	drive[frontRight].resetElapsedDistance();
	drive[backRight].resetElapsedDistance();
	Point2D delta = goal - robotPosition;

	while (delta.y > ROBOT_TOL) {
		// TODO: how to figure ideal v?
		double v = IDEAL_SPEED_DRIVE;

		double goal_orientation = atan2(-delta.x, delta.y);
            
		double error_angular = goal_orientation - robotAngle;
		if (error_angular > 0){
			error_angular = std::fmod(error_angular, M_PI);
		} else {
			error_angular = std::fmod(error_angular, -1*M_PI);
		}

		double w = angular_error_gain * error_angular;
	
		double leftWheelSpeed = calculate_wheel_speed(v, -1 * w);
		double rightWheelSpeed = calculate_wheel_speed(v, w);

		debug_log(std::string("Left wheel speed:") 
		+ std::to_string(leftWheelSpeed) 
		+ std::string(", right wheel speed: ")
		+ std::to_string(rightWheelSpeed));

		drive[frontLeft].setIdealSpeed(leftWheelSpeed);
		drive[backLeft].setIdealSpeed(leftWheelSpeed);
		drive[frontRight].setIdealSpeed(rightWheelSpeed);
		drive[backRight].setIdealSpeed(rightWheelSpeed);

		readEncoderVals();

		drive[frontLeft].update(encoderVal[frontLeft]);
		drive[backLeft].update(encoderVal[backLeft]);
		drive[frontRight].update(encoderVal[frontRight]);
		drive[backRight].update(encoderVal[backRight]);

		updateRobotOrientation();

		delta = goal - robotPosition;
	}

	drive[frontLeft].setIdealSpeed(0);
	drive[backLeft].setIdealSpeed(0);
	drive[frontRight].setIdealSpeed(0);
	drive[backRight].setIdealSpeed(0);

	debug_log(std::string("Robot Reached Goal"));
}

uint16_t Robot::getServoMotorEncoderVal(ServoMotor motor) {
	readEncoderVals();
	return encoderVal[4 + motor];
}

uint16_t Robot::getDriveMotorEncoderVal(DriveMotor motor) {
	readEncoderVals();
	return encoderVal[motor];
}

uint8_t Robot::getLimitVals() {
	readLimitVals();
	return limitVals;
}

bool Robot::getLimitVal(LimitSwitch s) {
	readLimitVals();
	return ((limitVals & (1 << s)) != 0);
}

// ugly but required by our confusing (for this) conventions
std::optional<LimitSwitch> Robot::determineLimitSwitch(ServoMotor m, bool positive) {
	switch (m) {
		case x:
			if (positive) {return {x_max};}
			return {x_min};
		case y:
			if (positive) {return {y_max};}
			return {y_min};
		default:
			// z
			if (positive) {return {z_max};}
			return {};
	}
}

void Robot::resetServoArm(ServoMotor motor) {
	// waits to hit the limit switch

	// use minimum limit switch for xy and max for z
	std::optional<LimitSwitch> limit = determineLimitSwitch(motor, (motor == z));

	servoArm[motor].simple_bkwd();

	while (!getLimitVal(*limit)) {}

	servoArm[motor].simple_stop();
	armPosition[motor] = 0;
	servoArm[motor].resetElapsedDistance();

	debug_log(std::string("Arm Reached Goal"));
}

void Robot::moveServoArm(ServoMotor motor, double pos) {
	double delta = pos - armPosition[pos];
	
	std::optional<LimitSwitch> limit = determineLimitSwitch(motor, (delta > 0));

	if (delta > 0) {
		servoArm[motor].simple_go();
	} else {
		servoArm[motor].simple_bkwd();
	}

	resetEncoderVals();
	servoArm[motor].resetElapsedDistance();

	while (delta < ARM_TOL) {
		readEncoderVals();

		// limit is some (not z min) and the value is true
		if (limit && getLimitVal(*limit)) {
			log(std::string("DEBUG: Hit limit switch in moveServoArm()."));
			break;
		}

		// 4 bc 4 drive motors
		updateArmPosition();
		delta = pos - armPosition[motor];
	}
	servoArm[motor].simple_stop();
}

void Robot::pollinate() { 
	double delta = 2.5;
	Point3D currPos = getArmPosition();
	moveServoArm(x, currPos.x - delta);
	moveServoArm(y, currPos.y - delta);
	moveServoArm(x, currPos.x + delta);
	moveServoArm(y, currPos.y + delta);
	moveServoArm(x, currPos.x);
	moveServoArm(y, currPos.y);
}

std::vector<Point3D> Robot::scan() {
	std::vector<Point3D> flowersToVisit;
	for (int i = 1; i < 4; i++){
		for (int j = 1; j < 3; j++){
			moveServoArm(x, CARTESIAN_X_MAX*i/4);
			moveServoArm(y, CARTESIAN_Y_MAX*j/3);
			std::vector<Point3D> newFlowers = findFlowers();
			flowersToVisit.insert(flowersToVisit.end(), newFlowers.begin(), newFlowers.end());
		}
	}
	return avgClusterCenters(flowersToVisit, 10);
}

std::vector<Point3D> Robot::findFlowers(){
	camera.storeSnapshot();
	cv::Mat image = camera.getColorImage();
	if (DEBUG) {
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imwrite(std::string("./plots/") + getFormattedTimeStamp() + std::string("_image.png"), image);
		cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
	}
	std::vector<Point2D> flowerCenters = findFlowerCenters(image);
	if (DEBUG) {
		for (Point2D const& blob : flowerCenters) {
			cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
			log(std::string("depth val: ") + std::to_string(camera.getDepthVal(x, y)));
		}

		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imwrite(std::string("./plots/") + getFormattedTimeStamp() + std::string("_yellow_blobs.png"), image);
		cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
	}
	flowerCenters = avgClusterCenters(flowerCenters, 10);
	if (DEBUG) {
		for (Point2D const& blob : flowerCenters) {
			cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
		}
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imwrite(std::string("./plots/") + getFormattedTimeStamp() + std::string("_clustered.png"), image);
		cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
	}
	
	std::vector<Point3D> cam3DPoints = camera.getDeprojection(flowerCenters);
	Point3D armPosition = getArmPosition();
	return camera2robot(cam3DPoints, armPosition.x, armPosition.y);
}

void Robot::pollinate_row(int n) {
	log(std::string("INFO robot: starting row."));
	for (int i = 0; i < n; i++) {
		log(std::string("INFO robot: driving forward one robot length."));
		// drive forward one robot length
		driveRobotForward(Point2D{-robotPosition.x, CARTESIAN_Y_MAX});
		
		// scan for flowers
		std::vector<Point3D> flowers = scan();
		for ( Point3D flower : flowers ) {
			// check if point is within bounds
			if (flower.x > CARTESIAN_X_MAX || flower.x < CARTESIAN_X_MIN ||
				flower.y > CARTESIAN_Y_MAX || flower.y < CARTESIAN_Y_MIN ||
				flower.z > CARTESIAN_Z_MAX || flower.z < CARTESIAN_Z_MIN) {
				std::stringstream ss;
				ss << "INFO robot: ignoring out of bound flower at: (" 
				   << flower.x << ", " << flower.y << ", " << flower.x << ").";
				log(ss.str());
				continue;
			}
			// move above plant
			moveServoArm(x, flower.x);
			moveServoArm(y, flower.y);

			// take another picture for precision?

			std::stringstream ss;
			ss << "INFO robot: polinating flower at: (" << flower.x 
			   << ", " << flower.y << ", " << flower.x << ").";
			log(ss.str());
			// move down and pollinate
			moveServoArm(z, flower.z);
			pollinate();

			// move back up
			moveServoArm(z, CARTESIAN_Z_MAX);
		}

		// move arm back to 0,0,high
		moveServoArm(x, 0);
		moveServoArm(y, 0);
		moveServoArm(z, CARTESIAN_Z_MAX);
	}
	log(std::string("INFO robot: done row!"));
}
