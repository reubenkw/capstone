#ifndef FRAME_TRANSFORM_H
#define FRAME_TRANSFORM_H

#include "point.h"
#include <vector>

Point3D camera2robot(Point3D camera_point, double arm_x, double arm_y);
std::vector<Point3D> camera2robot(std::vector<Point3D> camera_points, double arm_x, double arm_y);
Point3D robot2global(Point3D robot_point, double robot_x, double robot_y, double robot_angle);

#endif //h 
