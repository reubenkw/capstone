#include "cluster.h"
#include "point.h"

#include <math.h>
#include <vector>
#include <assert.h>
#include <algorithm>
#include <ostream>
#include <iostream>

using namespace std;

bool fits_cluster(vector<Point3D> cluster, Point3D point, float d) {
    for (int i = 0; i < cluster.size(); i++) {
        // check distance
        if ((point - cluster[i]).mag() < d) {
            return true;
        }
    }
    return false;
}

vector<vector<Point3D>> cluster(vector<Point3D> points, float d) {
    vector<vector<Point3D>> clusters{};
    
    for (auto point : points) {
        std::vector<int> fits_clusters;
        for (int i = 0; i < clusters.size(); i++) {
            if (fits_cluster(clusters[i], point, d)) {
                fits_clusters.push_back(i);
            }
        }
        if (fits_clusters.empty()) {
            clusters.push_back({point});
            continue;
        }
        
        int prnt_cluster = fits_clusters.front();
        clusters[prnt_cluster].push_back(point);
        
        // Combine matched clusters
        for (auto it = fits_clusters.rbegin(); it != fits_clusters.rend(); ++it) {
            int index = *it;
            if (index != prnt_cluster) {
                clusters[prnt_cluster].insert(clusters[prnt_cluster].end(), clusters[index].begin(), clusters[index].end());
                clusters.erase(clusters.begin() + index);
            }
        }
    }
    
    return clusters;
}

// couple extra linear passes to convert between Point2D and Point3D
vector<vector<Point2D>> cluster(vector<Point2D> points, float d) {
    // convert from 2d to 3d
    vector<Point3D> pts3d;
    for ( Point2D pt2d : points ) {
        pts3d.push_back(Point3D(pt2d.x, pt2d.y, 0));
    }

    // cluster in 3d
    vector<vector<Point3D>> clustered = cluster(pts3d, d);

    // 3d to 2d
    vector<vector<Point2D>> clustered2d;
    for ( vector<Point3D> group3d : clustered ) {
        vector<Point2D> group2d;
        for ( Point3D pt3d : group3d ) {
            group2d.push_back(Point2D(pt3d.x, pt3d.y));
        }
        clustered2d.push_back(group2d);
    }
    return clustered2d;
}

vector<Point3D> avgClusterCenters(vector<Point3D> points, float d){
    vector<Point3D> avgClusterCenters;
    vector<vector<Point3D>> clusters = cluster(points, d);
    for(auto cluster : clusters){
        Point3D sum = (0, 0);
        for (auto point : cluster){
            sum = sum + point;
        }
        avgClusterCenters.push_back(Point3D{sum.x/cluster.size(), sum.y/cluster.size(), sum.z/cluster.size()});
    }
    return avgClusterCenters;
}

vector<Point2D> avgClusterCenters(vector<Point2D> points, float d){
    vector<Point2D> avgClusterCenters;
    vector<vector<Point2D>> clusters = cluster(points, d);
    for(auto cluster : clusters){
        Point2D sum = (0, 0);
        for (auto point : cluster){
            sum = sum + point;
        }
        avgClusterCenters.push_back(Point2D{sum.x/cluster.size(), sum.y/cluster.size()});
    }
    return avgClusterCenters;
}

void test_clustering() {
    vector<Point2D> points{Point2D{.1, .1}, Point2D{.1, .2}, Point2D{.2, .3}, Point2D{0, 1}, Point2D{0, .9}, Point2D{1, 1}};
    vector<vector<Point2D>> expected{{Point2D{1, 1}}, {Point2D{0, .9}, Point2D{0, 1}}, {Point2D{.1, .1}, Point2D{.1, .2}, Point2D{.2, .3}}};
    vector<vector<Point2D>> recieved = cluster(points, 0.2);

    assert(recieved.size() == 3);

    // sort each group
    for (int i = 0; i < 3; i++) {
        sort(recieved[i].begin(), recieved[i].end(), [](auto &left, auto &right) {
            if (left.x == right.x) {
                return left.y < right.y;
            }
            return left.x < right.x;
        });
    }

    // sort groups based on length
    sort(recieved.begin(), recieved.end(), [](auto &left, auto &right) {return left.size() < right.size();});
    for (int i = 0; i < 3; i++) {
        assert(recieved[i].size() == expected[i].size());
        for (int j = 0; j < expected[i].size(); j++) {
            assert(recieved[i][j].x == expected[i][j].x);
            assert(recieved[i][j].y == expected[i][j].y);
        }
    }
}
