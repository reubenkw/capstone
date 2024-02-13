#include "pid_ctrl.h"
#include "imaging.h"
#include "log.h"
#include "comm.h"
#include "cluster.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/core/types.hpp>

std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

int image_processing_test() {
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

	return 0;
}

int pid_motor_ctrl_test() {
	PID_Ctrl pid_ctrl(10, 1, 2, 5);
    
    std::ofstream myfile;
    myfile.open ("test_pid_ctrl.csv");
    for (int i = 20; i > 0; i--){
        myfile << pid_ctrl.update_ctrl_signal(i, 0.1) << ",";
    }
    myfile.close();

	return 0;
}

int test_camera_image() {
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
	return 0;
}

int test_limit_switch(){
	unsigned int pin = 15;
	initialize_gpio(pin);
	int val = read_gpio(pin);
	log(std::string("gpio val: ") + std::to_string(val));
	return 0;
}

int main(int argc, char** argv)
{
	clear_log();
	log(std::string("Starting Program!"));
	// return test_camera_image();
	// return image_processing_test();
	// test_clustering();
	return test_limit_switch();
}
