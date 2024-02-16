#include "robot.h"

#include "log.h"
#include "cluster.h"
#include "constants.h"
#include "frame_transform.h"

#include <cmath>
#include <unistd.h> // linux sleep

#define IDEAL_LINEAR_SPEED 0.1

Robot::Robot(Camera & camera) 
    : camera(camera){
    
	initialize_log();
	readEncoderVals();
	i2c_bus_file = open_i2c();
	
	// TODO: pid terms need to be determined experimentally
    drive[frontLeft] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, frontLeft, encoderVal[frontLeft], i2c_bus_file);
	drive[backLeft] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, backLeft, encoderVal[backLeft], i2c_bus_file);
    drive[frontRight] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, frontRight, encoderVal[frontRight], i2c_bus_file);
    drive[backRight] = MotorController(10, 1, 1, 5, 0, 0, DRIVE_MC, backRight, encoderVal[backRight], i2c_bus_file);

	// 4 bc 4 drive motors
	servoArm[x] = MotorController(10, 1, 1, 5, 0, 0, SERVO_MC, x, encoderVal[4 + x], i2c_bus_file);
	servoArm[y] = MotorController(10, 1, 1, 5, 0, 0, SERVO_MC, y, encoderVal[4 + y], i2c_bus_file);
	servoArm[z] = MotorController(10, 1, 1, 5, 0, 0, SERVO_MC, z, encoderVal[4 + z], i2c_bus_file);

	robotPosition = Point2D(0, 0);
	robotAngle = 0;
	// arm starts in upright position
	armPosition = Point3D(0, 0, CARTESIAN_Z_MAX);
}

Point2D Robot::getRobotPosition() {
	return robotPosition;
}

double Robot::getRobotAngle() {
	return robotAngle;
}

Point3D Robot::getArmPosition() {
	return armPosition;
}

void Robot::readEncoderVals(){
	read_i2c(i2c_bus_file, MCU_ENCODER, (uint8_t * )encoderVal, 14);
}

void Robot::readLimitVals(){
	read_i2c(i2c_bus_file, MCU_1, &limitVal, 1);
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
}

void Robot::updateArmPosition() {
	armPosition[0] = servoArm[0].getElapsedDistance();
	armPosition[1] = servoArm[1].getElapsedDistance();
	armPosition[2] = servoArm[2].getElapsedDistance();
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
	drive[frontLeft].resetElapsedDistance();
	drive[backLeft].resetElapsedDistance();
	drive[frontRight].resetElapsedDistance();
	drive[backRight].resetElapsedDistance();
	Point2D delta = goal - robotPosition;

	while (delta.mag() < ROBOT_TOL) {
		// TODO: how to figure ideal v?
		double v = IDEAL_LINEAR_SPEED;

		double goal_orientation = atan2(delta.y, delta.x);
            
		double error_angular = std::abs(std::fmod(goal_orientation - robotAngle, M_PI));

		double w = angular_error_gain * error_angular;
	
		double leftWheelSpeed = calculate_wheel_speed(v, -1 * w);
		double rightWheelSpeed = calculate_wheel_speed(v, w);

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

}

uint16_t Robot::getServoMotorEncoderVal(ServoMotor motor) {
	readEncoderVals();
	return encoderVal[4 + motor];
}

uint16_t Robot::getDriveMotorEncoderVal(DriveMotor motor) {
	readEncoderVals();
	return encoderVal[motor];
}

uint8_t Robot::getLimitVal() {
	readLimitVals();
	return limitVal;
}

void Robot::resetServoArm(ServoMotor motor) {
	bool limitSwitch = false;
	// TODO: read limit switch values servo motor
	// TODO: figure out direction
	double idealSpeed = IDEAL_LINEAR_SPEED;

	servoArm[motor].setIdealSpeed(idealSpeed);

	while (!limitSwitch) {
		readLimitVals();
		limitSwitch = limitVal & (0x1 << motor * 2);
	}
	servoArm[motor].setIdealSpeed(0);
	armPosition[motor] = 0;
	servoArm[motor].resetElapsedDistance();
}

void Robot::moveServoArm(ServoMotor motor, double pos) {
	double delta = pos - armPosition[pos];
	// TODO: how to figure ideal v?
	double idealSpeed = IDEAL_LINEAR_SPEED;

	// TODO: do we need PID controllers ideal speed of servo arm position? 
	servoArm[motor].setIdealSpeed(idealSpeed);

	while (delta < ARM_TOL) {
		readEncoderVals();

		// TOOD: read limit switches

		// 4 bc 4 drive motors
		servoArm[motor].update(encoderVal[4 + motor]);
		updateArmPosition();
		delta = pos - armPosition[motor];
	}
	servoArm[motor].setIdealSpeed(0);
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
