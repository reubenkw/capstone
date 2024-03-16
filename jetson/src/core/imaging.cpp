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
bool isColor(Pixel pix, Pixel ideal, double angleTol, double lengthTol) {
	
	double dp = dotProduct(pix, ideal);
	return acos(dp / sqrt(dotProduct(pix, pix)) / sqrt(dotProduct(ideal, ideal))) < angleTol &&
			std::abs(toGreyscale(ideal) - toGreyscale(pix)) < lengthTol;
	
	return false;
}

cv::Mat colorMask(cv::Mat& image, Pixel ideal, double angleTol, double lengthTol) {
	cv::Mat thresholded =  cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);;

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			Pixel pix{ p->x, p->y, p->z };
			if (isColor(pix, ideal, angleTol, lengthTol)) {
				thresholded.at<uchar>(i, j) = 255;
			}
		}
	}

	return thresholded;
}

std::vector<Point2D> findFlowerCenters(cv::Mat& image){
	std::vector<Point2D> yellowBlobs;

	cv::Mat yellowMask = colorMask(image, {255, 255, 255}, 0.1, 75);
	cv::imwrite("./plots/yellow.png", yellowMask);

	cv::Mat green = colorMask(image, {82, 105, 74}, 0.1, 50);
	cv::imwrite("./plots/green.png", green);
	cv::blur(green, green, cv::Size(20, 20));
	cv::imwrite("./plots/blurredGreen.png", green);

	cv::Mat labels, stats, centroids;
	int label_count = cv::connectedComponentsWithStats(yellowMask, labels, stats, centroids);
	
	// start index at 1 since first blob is background blob
	// TODO: smarter way to determine hardcoded cutoffs?
	for (int i = 1; i < label_count; i++) {
		int blurredGreenVal = green.at<uchar>((int)centroids.at<double>(i, 1), (int)centroids.at<double>(i, 0));
		if ( blurredGreenVal > 10) {
			yellowBlobs.push_back({ centroids.at<double>(i, 0), centroids.at<double>(i, 1) });
		}
	}
	return yellowBlobs;
}

// TODO: Find center of row
double findYCenterOfPlant(cv::Mat& image) {
	return 0;
}

Camera::Camera() : color{rs2::frame()}, depth{rs2::frame()} {
	log(std::string("INFO Camera: starting init"));
	p.start();

	// Block program until frames arrive
	storeSnapshot();
	log(std::string("INFO Camera: first frame read"));

	// set camera intrinsic properties
	rs2_error *e = 0;
	rs2::video_stream_profile profile(depth.get_profile());
	intrinsic = profile.get_intrinsics();
	log(std::string("INFO Camera: init done"));
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
	const int w = depth.as<rs2::video_frame>().get_width();
    const int h = depth.as<rs2::video_frame>().get_height();
    // Create OpenCV matrix of size (w,h) from the depth data

    return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);;
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
	// x,y are percent
	// Get the depth frame's dimensions
	float width = depth.get_width();
	float height = depth.get_height();

	// perhaps some alignment should be done? 
	// https://github.com/IntelRealSense/librealsense/blob/master/examples/align/rs-align.cpp
	return depth.get_distance(std::floor(width * x), std::floor(height * y));
}

Point3D Camera::getDeprojection(Point2D color_pixel) {
	float width = color.get_width();
	float height = color.get_height();

	double d = this->getDepthVal(color_pixel.x / width, color_pixel.y / height);

	float point[3];
	float pixel[2] = {float(color_pixel.x), float(color_pixel.y)};
	rs2_deproject_pixel_to_point(point, &intrinsic, pixel, d);

	return Point3D(point[0], point[1], point[2]);
}

std::vector<Point3D> Camera::getDeprojection(std::vector<Point2D> const & color_pixels){
	std::vector<Point3D> cameraPoints;
	for (auto pixel: color_pixels){
		cameraPoints.push_back(getDeprojection(pixel));
	}
	return cameraPoints;
}
