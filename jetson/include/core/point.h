#ifndef POINT_H
#define POINT_H

#include <stdexcept>
#include <math.h> 

struct Point2D {
	double x;
	double y;

	Point2D(double x = 0, double y = 0) : x(x), y(y) {}
	bool operator==(Point2D const& rhs) { return x == rhs.x && y == rhs.y; }
	bool operator!=(Point2D const& rhs) { return !(*this == rhs); }
	Point2D operator+(Point2D const& rhs) { return Point2D(x + rhs.x, y + rhs.y); }
	Point2D operator-(Point2D const& rhs) { return Point2D(x - rhs.x, y - rhs.y); }
	double& operator[](int index) {
		if (index == 0) return x;
		else if (index == 1) return y;
		else throw std::out_of_range("Index out of range");
	}
	double mag() { return sqrt(x * x + y * y); }
};

struct Point3D {
	double x;
	double y;
	double z;

	Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
	bool operator==(Point3D const& rhs) { return x == rhs.x && y == rhs.y && z == rhs.z; }
	bool operator!=(Point3D const& rhs) { return !(*this == rhs); }
	Point3D operator+(Point3D const& rhs) { return Point3D(x + rhs.x, y + rhs.y, z + rhs.z); }
	Point3D operator-(Point3D const& rhs) { return Point3D(x - rhs.x, y - rhs.y, z - rhs.z); }
	double& operator[](int index) {
		if (index == 0) return x;
		else if (index == 1) return y;
		else if (index == 2) return z;
		else throw std::out_of_range("Index out of range");
	}
	double mag() { return sqrt(x * x + y * y + z * z); }
};

#endif
