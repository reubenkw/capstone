#ifndef IMAGING_H
#define IMAGING_H

#include "point.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>


std::vector<Point> findFlowerCenters(cv::Mat &  image);
double findYCenterOfPlant(cv::Mat & image);

struct Pixel {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

struct Image {
	int height;
	int width;
	Pixel * pixels;
};

// TODO: conversion between Image returned from Camera and cv::Mat

class Camera {
	rs2::pipeline p;
	rs2::video_frame color;
	rs2::depth_frame depth;

public:
	Camera();
	void storeSnapshot();
	Image getCameraImage();
	double getDepthVal(float x, float y);
};

#endif // h