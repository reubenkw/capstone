#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>

// TODO: refactor to have a 2D point and a 3D point in a seperate file
struct ClPoint {
    float x;
    float y;
};

std::vector<std::vector<ClPoint>> cluster(std::vector<ClPoint> points, float d);
void test_clustering();

#endif //h 
