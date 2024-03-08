#include "pid_ctrl.h"
#include "imaging.h"
#include "log.h"
#include "comm.h"
#include "cluster.h"
#include "robot.h"
#include "constants.h"

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/core/types.hpp>

std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

void image_processing_test() {
	Camera cam;

	cv::Mat image = cam.getColorImage();

	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::imwrite("./plots/original_image.png", image);
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

	std::vector<Point2D> yellowBlobs = findFlowerCenters(image);

	cv::Mat depth = cam.getDepthImage();
	cv::imwrite("./plots/depth.png", depth);
	int width = image.cols;
	int height = image.rows;

	for (Point2D const& blob : yellowBlobs) {
		cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
		float x = blob.x/width;
		float y = blob.y/height;
		log(std::string("depth val: ") + std::to_string(cam.getDepthVal(x, y)));
	}
	
	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::imwrite("plots/flowers.jpg", image);
}

void pid_motor_ctrl_test() {
	PID_Ctrl pid_ctrl(10, 1, 2, 5);
    
    std::ofstream myfile;
    myfile.open ("test_pid_ctrl.csv");
    for (int i = 20; i > 0; i--){
        myfile << pid_ctrl.update_ctrl_signal(i, 0.1) << ",";
    }
    myfile.close();
}

void test_camera_image() {
	log(std::string("Starting test_camera_image"));
	Camera cam;

	cv::Mat color = cam.getColorImage();
	log(std::string("color image"));
	cv::Mat depth = cam.getDepthImage();
	log(std::string("depth image"));

	cv::Mat out = color;
	cv::cvtColor(color, out, cv::COLOR_RGB2BGR);

	cv::imwrite("./plots/color.png", color);
	log(std::string("wrote color image"));

	cv::imwrite("./plots/depth.png", depth);
	log(std::string("wrote depth image"));

	log(std::string("saved color and depth image under jetson/plots/"));
}

void test_i2c_read() {
	int i2c_bus_file = open_i2c();
	log(std::string("INFO: bus file: ") + std::to_string(i2c_bus_file));
	uint8_t data[4] = {0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
		read_i2c(i2c_bus_file, 0x42, (uint8_t * )data, 4);
	}
}

void test_i2c_write() {
	int i2c_bus_file = open_i2c();
	log(std::string("INFO: bus file: ") + std::to_string(i2c_bus_file));
	uint8_t data[4] = {0x5, 0x0, 0x0, 0x0};
	write_i2c(i2c_bus_file, 0x42, data, 4);
}

void test_scan() {
	log(std::string("INFO: starting test_scan."));
	Camera cam;
	Robot r(cam);
	std::vector<Point3D> flowers = r.scan();
	for ( Point3D flower : flowers ) {
		log(std::string("flower detected: " + std::to_string(flower.x) + ", " 
			+ std::to_string(flower.y) + ", " + std::to_string(flower.z)));
	}
	log(std::string("INFO: done test_scan."));
}

void test_pollinate() {
	log(std::string("INFO: starting test_pollinate."));
	Camera cam;
	Robot r(cam);
	r.pollinate();
	log(std::string("INFO: done test_pollinate."));
}

void test_drive() {
	log(std::string("INFO: starting test_drive."));
	Camera cam;
	Robot r(cam);
	Point2D delta{0, CARTESIAN_Y_MAX};
	r.driveRobotForward(delta);
	log(std::string("INFO: done test_drive."));
}

void test_arm_x() {
	log(std::string("INFO: starting test_arm_x"));
	Camera cam;
	Robot r(cam);
	r.resetServoArm(x);
	r.moveServoArm(x, CARTESIAN_X_MAX);
	log(std::string("done testing test_arm_x"));
}

void test_arm_y() {
	log(std::string("INFO: starting test_arm_y"));
	Camera cam;
	Robot r(cam);
	r.resetServoArm(y);
	r.moveServoArm(y, CARTESIAN_Y_MAX);
	log(std::string("done testing test_arm_y"));
}

void test_arm_z() {
	log(std::string("INFO: starting test_arm_z"));
	Camera cam;
	Robot r(cam);
	r.resetServoArm(z); 
	r.moveServoArm(z, CARTESIAN_Z_MIN);
	log(std::string("done testing test_arm_z"));
}

void test_limit_switches() {
	log(std::string("INFO: starting test_limit_switches (expecting user input)."));
	Camera cam;
	Robot r(cam);
	int n_state_changes = 10;
	int8_t limits = 0;
	int8_t limits_prev = 0;
	while(true) {
		limits = r.getLimitVals();
		if (limits != limits_prev) {
			log(std::string("INFO: limit state changed. Value: " + limits));
		}
		limits_prev = limits;
	}

	log(std::string("INFO: done test_limit_switches (check log)."));
}

void test_mc() {
	int file = open_i2c();
	MotorController mc(0, 0, 0.01, 0, 0.01, 0, SERVO_MC, x, 0, file);

	while(true) {
		// forward 5 sec
		mc.simple_go();
		usleep(5000000);

		// stop 2 sec
		mc.simple_stop();
		usleep(2000000);

		// back 5 sec
		mc.simple_bkwd();
		usleep(5000000);

		// stop 2 sec
		mc.simple_stop();
		usleep(2000000);
	}
}

int main(int argc, char** argv)
{
	initialize_log();
	log(std::string("Starting Program!"));
	// test_camera_image();
	// image_processing_test();
	// test_clustering();
	// test_i2c_read();
	test_mc();
	return 0;
}
