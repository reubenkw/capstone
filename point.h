#ifndef POINT_H
#define POINT_H

#include <stdexcept>
#include <math.h> 

struct Point {
	double x;
	double y;
	double z;

	Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
	bool operator==(Point const& rhs) { return x == rhs.x && y == rhs.y && z == rhs.z; }
	bool operator!=(Point const& rhs) { return !(*this == rhs); }
	Point operator+(Point const& rhs) { return Point(x + rhs.x, y + rhs.y, z + rhs.z); }
	Point operator-(Point const& rhs) { return Point(x - rhs.x, y - rhs.y, z - rhs.z); }
	double& operator[](int index) {
		if (index == 0) return x;
		else if (index == 1) return y;
		else if (index == 2) return z;
		else throw std::out_of_range("Index out of range");
	}
	double mag() { return sqrt(x * x + y * y + z * z); }
};

#endif
