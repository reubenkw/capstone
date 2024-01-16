#include "imaging.h"

#include <stdexcept>
#include <iostream>

// very rough outline of an imaging class

Pixel Image::readPixelVal(int x, int y) {
	if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT) {
		std::cout << "Pixel: " << x << ", " << y << " out of bounds";
		throw std::exception();
	}
	return image[x][y];
}

void Image::writePixelVal(int x, int y, Pixel val) {
	if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT) {
		std::cout << "Pixel: " << x << ", " << y << " out of bounds";
		throw std::exception();
	}
	image[x][y] = val;
}

// TODO: Camera Initialization
Camera::Camera() {
}

// TODO: Take a camera image
Image Camera::getCameraImage() {
	Image image;
	return image;
}

// TODO: Read depth val of pixel from camera
double Camera::getDepthVal(int x, int y) {
	return 0;
}
