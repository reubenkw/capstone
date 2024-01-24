#ifndef IMAGING_H
#define IMAGING_H

#include "point.h"

#include <vector>
#include <opencv2/opencv.hpp>

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
public:
	Camera();
	Image getCameraImage();
	double getDepthVal(int x, int y);
};

#endif // h