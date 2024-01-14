#include "pid_ctrl.h"
#include "imaging.h"

#include <stdio.h>
#include <iostream>
#include <fstream>

int image_processing_test() {
	cv::Mat image;
	image = cv::imread(".\\data\\flowers.jpg");
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
	cv::imshow("image", image);
	cv::waitKey(0);

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

int main(int argc, char** argv)
{
	return image_processing_test();
}