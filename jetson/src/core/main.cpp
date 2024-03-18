#include "pid_ctrl.h"
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

std::vector<Point3D> test_image_processing(Camera & cam) {
	cv::Mat image = cam.getColorImage();

	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	std::string tag = std::string("");
	cv::imwrite("./plots/original_image.png", image);
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
		
	log(std::string("finding yellow blobs!!!!!!"));
	std::vector<Point2D> yellowBlobs = findFlowerCenters(image, cam, tag);
	for (Point2D const& blob : yellowBlobs) {
		log(std::string("yellow blob") + std::to_string(blob.x) + std::string(",") + std::to_string(blob.y));
	}
	std::vector<Point2D> avgCenter = avgClusterCenters(yellowBlobs, 10);
	log(std::string("finding avgCenters!!!!!!"));
	for (Point2D const& blob : avgCenter) {
		log(std::string("avgCenter: ") + std::to_string(blob.x) + std::string(",") + std::to_string(blob.y));
	}

	cv::Mat depth = cam.getDepthImage();
	cv::imwrite("./plots/depth.png", depth);
	int width = image.cols;
	int height = image.rows;

	for (Point2D const& blob : avgCenter) {
		cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
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

void test_i2c_read(uint address) {
	int i2c_bus_file = open_i2c();
	log(std::string("INFO: bus file: ") + std::to_string(i2c_bus_file));
	uint8_t data[1] = {0};
	while(true) {
		read_i2c(i2c_bus_file, CMD_WRITE_STATUS, address, data, 1);
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
	Point2D delta{0, CARTESIAN_Y_MAX};
	r.driveRobotForward(delta);
	log(std::string("INFO: done test_drive."));
}

void test_arm_x() {
	log(std::string("INFO: starting test_arm_x"));
	Camera cam;
	Robot r(cam);
	r.moveServoArm(x, 0);
	r.moveServoArm(x, CARTESIAN_X_MAX);
	log(std::string("done testing test_arm_x"));
}

void test_arm_y() {
	log(std::string("INFO: starting test_arm_y"));
	Camera cam;
	Robot r(cam);
	r.moveServoArm(y, 0);
	r.moveServoArm(y, CARTESIAN_Y_MAX);
	log(std::string("done testing test_arm_y"));
}

void test_arm_z() {
	log(std::string("INFO: starting test_arm_z"));
	Camera cam;
	Robot r(cam);
	r.moveServoArm(z, 0.748); 
	r.moveServoArm(z, CARTESIAN_Z_MIN);
	log(std::string("done testing test_arm_z"));
}

void test_move_servo_arm(){
	Camera cam;
	Robot r(cam);
	printf("done init\n");
	r.moveServoArm(x, 0.05);
	r.moveServoArm(y, 0.05);
	r.moveServoArm(z, 0.7);
}

void test_move_servo_arm_to_flowers(){
	Camera cam;
	Robot r(cam);
	
	r.moveServoArm(z, 0.8);
	r.moveServoArm(x, 0);
	r.moveServoArm(y, 0);

	usleep(30000000);

	std::vector<Point3D> robotPoints = test_image_processing(cam);

	for (auto const & robotPoint : robotPoints){
		log(std::string("move arm to point: ") 
		+ std::to_string(robotPoint.x) + std::string(", ") 
		+ std::to_string(robotPoint.y) + std::string(", ")
		+ std::to_string(robotPoint.z));
		r.moveServoArm(x, robotPoint.x);
		r.moveServoArm(y, robotPoint.y);
		r.moveServoArm(z, robotPoint.z);
	}
	
}

void test_scan() {
	log(std::string("INFO: starting test_scan."));
	Camera cam;
	Robot r(cam);

	r.resetServoArm();

	std::vector<Point3D> flowerCenters = r.scan();

	for (auto const & flowerCenter : flowerCenters){

		// check if point is within bounds
		if (flowerCenter.x > CARTESIAN_X_MAX || flowerCenter.x < CARTESIAN_X_MIN ||
			flowerCenter.y > CARTESIAN_Y_MAX || flowerCenter.y < CARTESIAN_Y_MIN ||
			flowerCenter.z > CARTESIAN_Z_MAX || flowerCenter.z < CARTESIAN_Z_MIN) {
			std::stringstream ss;
			ss << "INFO robot: ignoring out of bound flower at: (" 
				<< flowerCenter.x << ", " << flowerCenter.y << ", " << flowerCenter.x << ").";
			log(ss.str());
			continue;
		}

		log(std::string("move arm to point: ") 
		+ std::to_string(flowerCenter.x) + std::string(", ") 
		+ std::to_string(flowerCenter.y) + std::string(", ")
		+ std::to_string(flowerCenter.z));

		usleep(1000000);

		r.moveServoArm(x, flowerCenter.x);
		r.moveServoArm(y, flowerCenter.y);
		r.moveServoArm(z, flowerCenter.z);

		usleep(1000000);
		r.moveServoArm(z, 0.8);

	}
	log(std::string("INFO: done test_scan."));
	
}

int main(int argc, char** argv)
{
	initialize_log();
	log(std::string("Starting Program!"));
	// test_camera_image();
	// Camera cam;
	// test_image_processing(cam);
	// test_clustering();
	// test_i2c_write();
	// test_i2c_read();
	// test_i2c_read_write();
	// test_move_servo_arm();
	// test_move_servo_arm_to_flowers();
	test_scan();
	// test_i2c_read_mcu_e();
	// test_move_servo_arm();
	return 0;
}
