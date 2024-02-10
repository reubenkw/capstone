#include "imaging.h"
#include "log.h"

#include <stdexcept>
#include <iostream>
#include <numeric>
#include <math.h>

#include <opencv2/core/types.hpp>
#include <librealsense2/rs.hpp>

// very rough outline of an imaging class

double toGreyscale(Pixel pix) {
	return (pix.r + pix.g + pix.b) / 3.0;
}

cv::Mat toGreyscale(cv::Mat& image) {
	cv::Mat greyscale = cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);;
	
	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			greyscale.at<uchar>(i, j) = (int) toGreyscale({ p->x, p->y, p->z });
		}
	}
	return greyscale;
}

double brightestPixelVal(cv::Mat& image) {
	double brightestPix = 0;
	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			double greyscale = toGreyscale({ p->x, p->y, p->z });
			if (greyscale > brightestPix) {
				brightestPix = greyscale;
			}
		}
	}
	return brightestPix;
}

double dotProduct(Pixel x, Pixel y) {
	return x.r * y.r + x.g * y.g + x.b * y.b;
}

// TODO: determine colour pixels better
bool isColor(Pixel pix, Pixel ideal, double brightestVal, double tol) {
	if (toGreyscale(pix) > brightestVal * 0.5) {
		double dp = dotProduct(pix, ideal);
		return acos(dp / sqrt(dotProduct(pix, pix)) / sqrt(dotProduct(ideal, ideal))) < tol;
	}
	return false;
}

cv::Mat colorMask(cv::Mat& image, double brightest, Pixel ideal, double tol) {
	cv::Mat thresholded =  cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);;

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			Pixel pix{ p->x, p->y, p->z };
			if (isColor(pix, ideal, brightest, tol)) {
				thresholded.at<uchar>(i, j) = 255;
			}
		}
	}

	return thresholded;
}

// TODO: Fancy schmancy image processing
std::vector<Point> findFlowerCenters(cv::Mat& image){
	std::vector<Point> yellowBlobs;

	double brightest = brightestPixelVal(image);

	cv::Mat yellowMask = colorMask(image, brightest, {255, 255, 0}, 0.25);
	cv::imwrite("./plots/yellow.png", yellowMask);
	cv::Mat white = colorMask(image, brightest, {255, 255, 255}, 0.1);
	cv::blur(white, white, cv::Size(100, 100));
	cv::imwrite("./plots/blurredWhite.png", white);

	cv::Mat labels, stats, centroids;
	int label_count = cv::connectedComponentsWithStats(yellowMask, labels, stats, centroids);

	// start index at 1 since first blob is background blob
	// TODO: smarter way to determine hardcoded cutoffs?
	for (int i = 1; i < label_count; i++) {
		if (white.at<uchar>((int)centroids.at<double>(i, 0), (int)centroids.at<double>(i, 1)) > 10) {
			yellowBlobs.push_back({ centroids.at<double>(i, 0), centroids.at<double>(i, 1) });
		}
	}

	// TODO: clustering algo

	return yellowBlobs;

}

// TODO: Find center of row
double findYCenterOfPlant(cv::Mat& image) {
	return 0;
}

// TODO: Camera Initialization
Camera::Camera() : color{rs2::frame()}, depth{rs2::frame()} {
	p.start();

	// Block program until frames arrive
	log(std::string("Waiting for camera init."));
	storeSnapshot();
	log(std::string("Camera initialized successfully"));
}

// returns the most recent snapshot
cv::Mat Camera::getColorImage() {
	// Query frame size (width and height)
    const int w = color.as<rs2::video_frame>().get_width();
    const int h = color.as<rs2::video_frame>().get_height();

    // Create OpenCV matrix of size (w,h) from the colorized depth data
    return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
}

// returns the most recent snapshot
cv::Mat Camera::getDepthImage() {
	rs2::colorizer c;

	rs2::video_frame colorized_depth = c.colorize(depth);
	log(std::string("colourized image"));

	// Query frame size (width and height)
    const int w = colorized_depth.get_width();
    const int h = colorized_depth.get_height();

    // Create OpenCV matrix of size (w,h) from the colorized depth data
    return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
}

// ensure that the color and depth image are associated
void Camera::storeSnapshot() {
	// blocks until frames have been recieved
	rs2::frameset frameset = p.wait_for_frames();

	depth = frameset.get_depth_frame();
	color = frameset.get_color_frame();
	
	// TODO: probably want some error handling
}

// Read depth val at a point in the image
double Camera::getDepthVal(float x, float y) {
	// Get the depth frame's dimensions
	float width = depth.get_width();
	float height = depth.get_height();

	// perhaps some alignment should be done? 
	// https://github.com/IntelRealSense/librealsense/blob/master/examples/align/rs-align.cpp
	return depth.get_distance(std::floor(width * x), std::floor(height * y));
}
