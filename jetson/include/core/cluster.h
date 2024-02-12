#ifndef CLUSTER_H
#define CLUSTER_H

#include "point.h"
#include <vector>

std::vector<std::vector<Point2D>> cluster(std::vector<Point2D> points, float d);
std::vector<std::vector<Point3D>> cluster(std::vector<Point3D> points, float d);
void test_clustering();

#endif //h 
