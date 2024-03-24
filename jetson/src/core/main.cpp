#include "imaging.h"
#include "log.h"
#include "comm.h"
#include "cluster.h"
#include "robot.h"
#include "constants.h"
#include "frame_transform.h"

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/core/types.hpp>

std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

void test_find_flowers(){
	cv::Mat image = cv::imread("./plots/original_image.png");

	std::string tag = std::string("");
	cv::imwrite("./plots/original_image.png", image);
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
		
	log(std::string("finding yellow blobs!!!!!!"));
	std::vector<Point2D> yellowBlobs = findFlowerCenters(image, tag);
	for (Point2D const& blob : yellowBlobs) {
		log(std::string("yellow blob") + std::to_string(blob.x) + std::string(",") + std::to_string(blob.y));
	}

	std::vector<Point2D> avgCenter = avgClusterCenters(yellowBlobs, 25);
	log(std::string("finding avgCenters!!!!!!"));
	for (Point2D const& blob : avgCenter) {
		cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
		log(std::string("avgCenter: ") + std::to_string(blob.x) + std::string(",") + std::to_string(blob.y));
	}

	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::imwrite("plots/" + tag + "_flowers.jpg", image);
}

std::vector<Point3D> test_image_processing(Camera & cam) {
	cv::Mat image = cam.getColorImage();

	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	std::string tag = std::string("");
	cv::imwrite("./plots/original_image.png", image);
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
		
	log(std::string("finding yellow blobs!!!!!!"));
	std::vector<Point2D> yellowBlobs = findFlowerCenters(image, tag);
	for (Point2D const& blob : yellowBlobs) {
		log(std::string("yellow blob") + std::to_string(blob.x) + std::string(",") + std::to_string(blob.y));
	}
	std::vector<Point2D> avgCenter = avgClusterCenters(yellowBlobs, 25);
	log(std::string("finding avgCenters!!!!!!"));
	for (Point2D const& blob : avgCenter) {
		cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
		log(std::string("avgCenter: ") + std::to_string(blob.x) + std::string(",") + std::to_string(blob.y));
	}

	cv::Mat depth = cam.getDepthImage();
	cv::imwrite("./plots/depth.png", depth);
	int width = image.cols;
	int height = image.rows;

	for (Point2D const& blob : avgCenter) {
		float x = blob.x/width;
		float y = blob.y/height;
		log(std::string("depth val: ") + std::to_string(cam.getDepthVal(x, y)));
	}

	std::vector<Point3D> cam3DPoints = cam.getDeprojection(avgCenter);
	for (Point3D const& blob : cam3DPoints) {
		log(std::string("cam point: ") 
		+ std::to_string(blob.x) + std::string(", ") 
		+ std::to_string(blob.y) + std::string(", ") 
		+ std::to_string(blob.z));
	}
	std::vector<Point3D> robotPoints = camera2robot(cam3DPoints, 0, 0);

	for (Point3D const& blob : robotPoints) {
		log(std::string("blob: ") 
		+ std::to_string(blob.x) + std::string(", ") 
		+ std::to_string(blob.y) + std::string(", ") 
		+ std::to_string(blob.z));
	}
	
	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::imwrite("plots/" + tag + "_flowers.jpg", image);

	return robotPoints;
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

void test_i2c_read(uint address) {
	int i2c_bus_file = open_i2c();
	log(std::string("INFO: bus file: ") + std::to_string(i2c_bus_file));
	uint8_t data[1] = {0};
	while(true) {
		read_i2c(i2c_bus_file, CMD_E_WRITE_STATUS, address, data, 1);
		usleep(1000000);
		printf("data: %x, %x, %x, %x\n", data[0], data[1], data[2], data[3]);
	}
}

void test_i2c_read_mcu_e() {
	test_i2c_read(MCU_E);
}

void test_i2c_read_mcu_m() {
	test_i2c_read(MCU_M);
}

void test_i2c_write(uint address) {
	int i2c_bus_file = open_i2c();
	log(std::string("INFO: bus file: ") + std::to_string(i2c_bus_file));
	uint8_t data[4] = {0x5, 0x0, 0x0, 0x0};
	write_i2c(i2c_bus_file, address, 0x1, data, 4);
}

void test_i2c_write_mcu_e() {
	test_i2c_write(MCU_E);
}

void test_i2c_write_mcu_m() {
	test_i2c_write(MCU_M);
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
	float drive_time = 3;
	uint8_t drive_pwm = 150;
	r.driveForwards(drive_pwm, drive_time);
	log(std::string("INFO: done test_drive."));
}

void test_arm_x() {
	log(std::string("INFO: starting test_arm_x"));
	Camera cam;
	Robot r(cam);
	r.setArmPosition(x, 0);
	r.setArmPosition(x, CARTESIAN_X_MAX);
	log(std::string("done testing test_arm_x"));
}

void test_arm_y() {
	log(std::string("INFO: starting test_arm_y"));
	Camera cam;
	Robot r(cam);
	r.setArmPosition(y, 0);
	r.setArmPosition(y, CARTESIAN_Y_MAX);
	log(std::string("done testing test_arm_y"));
}

void test_arm_z() {
	log(std::string("INFO: starting test_arm_z"));
	Camera cam;
	Robot r(cam);
	r.setArmPosition(z, 0.748); 
	r.setArmPosition(z, CARTESIAN_Z_MIN);
	log(std::string("done testing test_arm_z"));
}

void test_move_servo_arm(){
	Camera cam;
	Robot r(cam);
	printf("done init\n");
	r.setArmPosition(x, 0.05);
	r.setArmPosition(y, 0.05);
	r.setArmPosition(z, 0.7);
}

void test_move_servo_arm_to_flowers(){
	Camera cam;
	Robot r(cam);
	
	r.resetServoArm();
	sleep(1);

	std::vector<Point3D> robotPoints = test_image_processing(cam);

	for (auto const & robotPoint : robotPoints){
		// check if point is within bounds
		if (robotPoint.x > CARTESIAN_X_MAX || robotPoint.x < CARTESIAN_X_MIN ||
			robotPoint.y > CARTESIAN_Y_MAX || robotPoint.y < CARTESIAN_Y_MIN ||
			robotPoint.z > CARTESIAN_Z_MAX || robotPoint.z < CARTESIAN_Z_MIN) {
			std::stringstream ss;
			ss << "INFO robot: ignoring out of bound flower at: (" 
				<< robotPoint.x << ", " << robotPoint.y << ", " << robotPoint.x << ").";
			log(ss.str());
			continue;
		}
		log(std::string("move arm to point: ") 
		+ std::to_string(robotPoint.x) + std::string(", ") 
		+ std::to_string(robotPoint.y) + std::string(", ")
		+ std::to_string(robotPoint.z));
		r.setArmPosition(x, robotPoint.x);
		r.setArmPosition(y, robotPoint.y);
		r.setArmPosition(z, robotPoint.z);
	}
	
}

void test_drive_interface() {
	log(std::string("INFO: starting test_drive_interace."));
	Camera cam;
	Robot r(cam);

	while(true) {
		r.driveForwards(150, 5);
		sleep(2);
		r.driveBackwards(150, 5);
		sleep(2);
	}

	log(std::string("INFO: done test_drive_interace."));
	
}

void test_scan(){
	log(std::string("INFO: starting test_scan."));
	Camera cam;
	Robot r(cam);
	std::vector<Point3D> flowerCenters;
	// r.resetServoArm();
	sleep(5);
	cam.setExposure(50000); // lower for fake plant
	sleep(5);
	flowerCenters = r.scan();
	r.pollinate_all_in_zone(flowerCenters);	
	sleep(5);
	// r.resetServoArm();
	sleep(5);
}

void main_loop() {
	log(std::string("INFO: starting main_loop."));
	sleep(5);
	Camera cam;
	Robot r(cam);

	float drive_time = 7.5;
	uint8_t drive_pwm = 150; 
	float sleep_time = 2;

	std::vector<Point3D> flowerCenters;

	r.resetServoArm();
	sleep(5);

	while(true) {
		cam.setExposure(80000); // higher for real plant
		sleep(sleep_time);
		flowerCenters = r.scan();
		r.pollinate_all_in_zone(flowerCenters);
		sleep(sleep_time);
		r.resetServoArm();
		sleep(sleep_time);
		r.driveForwards(drive_pwm, drive_time);
		sleep(sleep_time);

			
		cam.setExposure(50000); // lower for fake plant
		sleep(sleep_time);
		flowerCenters = r.scan();
		r.pollinate_all_in_zone(flowerCenters);
		sleep(sleep_time);
		r.resetServoArm();
		sleep(sleep_time);
		r.driveBackwards(drive_pwm, drive_time);
		sleep(sleep_time);	
	}

	log(std::string("INFO: done main_loop."));
}

int main_loop(int argc, char** argv)
{
	initialize_log();
	log(std::string("Starting Program!"));
	test_scan();
	// main_loop();

	// test_drive_interface();
	// test_camera_image();
	// Camera cam;
	// test_image_processing(cam);
	// test_clustering();
	// test_i2c_write();
	// test_i2c_read();
	// test_i2c_read_write();
	// test_move_servo_arm();
	// test_move_servo_arm_to_flowers();
	//test_main_loop();
	// test_i2c_read_mcu_e();
	// test_move_servo_arm();
	return 0;
}
