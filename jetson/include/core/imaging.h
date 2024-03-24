#ifndef IMAGING_H
#define IMAGING_H

#include "point.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

struct Pixel {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

class Camera {
	rs2::pipeline p;
	rs2::context ctx;
	rs2::config cfg;
	rs2::video_frame color;
	rs2::depth_frame depth;
	rs2_intrinsics intrinsic;

public:
	Camera();
	void storeSnapshot();
	void setExposure(int exp);
	cv::Mat getColorImage();
	cv::Mat getDepthImage();
	double getDepthVal(float x, float y);
	Point3D getDeprojection(Point2D color_pixel);
	std::vector<Point3D> getDeprojection(std::vector<Point2D> const & color_pixels);
};

std::vector<Point2D> findFlowerCenters(cv::Mat &  image, std::string const & tag);
double findYCenterOfPlant(cv::Mat & image);

#endif // h