#include "pid_ctrl.h"
#include "imaging.h"
#include "log.h"
#include "comm.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/core/types.hpp>

std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

int image_processing_test() {
	cv::Mat image;
	image = cv::imread( "./data/flowers.jpg");
	cv::resize(image, image, cv::Size(), 0.25, 0.25);
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

	if (!image.data)
	{
		printf("No image data \n");
		return -1;
	}

	std::vector<Point> yellowBlobs = findFlowerCenters(image);

	for (Point const& blob : yellowBlobs) {
		cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 5, { 255, 0, 255 }, 5);
	}

	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
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
	cv::Mat depth = cam.getDepthImage();

	cv::imwrite("./plots/color.png", color);
	cv::imwrite("./plots/depth.png", depth);

	log(std::string("saved color and depth image under jetson/plots/"));
	return 0;
}

int main(int argc, char** argv)
{
	log(std::string("Starting Program!"));
	return test_camera_image();
}