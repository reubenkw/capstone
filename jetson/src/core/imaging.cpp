#include "imaging.h"
#include "log.h"

#include <stdexcept>
#include <iostream>
#include <numeric>
#include <math.h>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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

Pixel brightestPixelVal(cv::Mat& image, Point2D topLeft, double width, double height) {
	Pixel brightestPix = Pixel{0, 0, 0};
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i + topLeft.y, j + topLeft.x);
			double greyscale = toGreyscale({ (uint8_t) p->x, (uint8_t) p->y, (uint8_t) p->z });
			if (greyscale > toGreyscale(brightestPix)) {
				brightestPix = Pixel{(uint8_t)p->x, (uint8_t)p->y, (uint8_t)p->z};
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

cv::Mat whiteMask(cv::Mat& image, double brightest) {
	cv::Mat thresholded =  cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);;

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			// need double not uint8 bc values might go > 255 in the math
			double r = p->x;
			double g = p->y;
			double b = p->z;
			if (std::abs(r/g - 1.0) < 0.15 && // make sure red and green are close together
				std::abs(g/b - 1.0) < 0.5 && // make sure green and blue are close ish together
				std::abs(r/b - 1.0) < 0.5 && // make sure red and blue are close ish together
				(r + g + b) / 3 > 150 // make sure its relatively bright compared to the whole iamge
				) {
				thresholded.at<uchar>(i, j) = 255;
			}
		}
	}

	return thresholded;
}

cv::Mat greenMask(cv::Mat& image) {
	cv::Mat thresholded =  cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);;

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			// need double not uint8 bc values might go > 255 in the math
			double r = p->x;
			double g = p->y;
			double b = p->z;
			if (g > r * 1.1 && g > b * 1.1 && g > 55 && g < 180) {
				thresholded.at<uchar>(i, j) = 255;
			}
		}
	}

	return thresholded;
}

bool nearYellow(cv::Mat& image, cv::Mat& yellow, Point2D topLeft, int width, int height, double area){
	Pixel brightest = brightestPixelVal(image, topLeft, width, height);
	log(std::string("brightest: ") + 
		std::to_string(brightest.r) + 
		std::string(" ") + 
		std::to_string(brightest.g) +
		std::string(" ") + 
		std::to_string(brightest.b) );

	int startY = std::max(0, int(topLeft.y - 2*height));
	int startX = std::max(0, int(topLeft.x - 2*width));

	int endY = std::min(image.rows, int(topLeft.y + 2*height));
	int endX = std::min(image.cols, int(topLeft.x + 2*width));

	log(std::to_string(startY) + std::string(" ") + std::to_string(startX) );
	log(std::to_string(endY) + std::string(" ") + std::to_string(endX) );

	int numYellow = 0;
	for (int i = startY; i < endY; i++) {
		for (int j = startX; j < endX; j++) {
			cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(i, j);
			// need double not uint8 bc values might go > 255 in the math
			double r = p->x;
			double g = p->y;
			double b = p->z;
			if ((double)brightest.b/brightest.r - b/r > 0.2 &&  // make sure there is less blue in the ratio compared to white
				std::abs(r/g - 1.0) < 0.15 && // make sure red and green are close together
			    (r + g)/2 > (brightest.r + brightest.g) / 2 * 0.65 // make sure its bright enough
				) {
				yellow.at<uchar>(i, j) = 255;
				numYellow++;
			}
		}
	}
	log(std::string("numYellow: ") + std::to_string(numYellow));
	log(std::string("area: ") + std::to_string(area));
	return numYellow > 5;
}

std::vector<Point2D> findFlowerCenters(cv::Mat& image, std::string const & tag){
	std::vector<Point2D> whiteBlobs;
	cv::Mat yellow =  cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);

	Pixel brightest = brightestPixelVal(image, {0,0}, image.cols, image.rows);

	cv::Mat white = whiteMask(image, toGreyscale(brightest));
	cv::dilate(white, white, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::imwrite("./plots/" + tag + "_white.png", white);

	cv::Mat labels, stats, centroids;
	int label_count = cv::connectedComponentsWithStats(white, labels, stats, centroids);
	
	// start index at 1 since first blob is background blob
	// TODO: smarter way to determine hardcoded cutoffs?
	int width = image.cols;
	int height = image.rows;

	for (int i = 1; i < label_count; i++) {
		double centerX = centroids.at<double>(i, 0);
		double centerY = centroids.at<double>(i, 1);
		float x = centerX/width;
		float y = centerY/height;
		if (stats.at<int>(i, cv::CC_STAT_AREA) > 10 && 
			nearYellow(image, yellow, 
			Point2D(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP)), 
			stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT), 
			stats.at<int>(i, cv::CC_STAT_AREA)) // check that the white petal is near yellow
			) { 
			whiteBlobs.push_back({ centroids.at<double>(i, 0), centroids.at<double>(i, 1) });
		}
	}

	cv::imwrite("./plots/" + tag + "_yellow.png", yellow);

	return whiteBlobs;
}

// TODO: Find center of row
double findYCenterOfPlant(cv::Mat& image) {
	return 0;
}

Camera::Camera() : color{rs2::frame()}, depth{rs2::frame()} {
	log(std::string("INFO Camera: starting init"));
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720);
	p.start(cfg);
	
	// set manual exposure
	auto colorSensor = p.get_active_profile().get_device().query_sensors()[0];
	colorSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
	colorSensor.set_option(RS2_OPTION_EXPOSURE, 50000);

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

void Camera::setExposure(int exp) {
	// set manual exposure
	auto colorSensor = p.get_active_profile().get_device().query_sensors()[0];
	colorSensor.set_option(RS2_OPTION_EXPOSURE, exp);
	storeSnapshot();
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
