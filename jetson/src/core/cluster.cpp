#include "cluster.h"

#include <math.h>
#include <vector>
#include <assert.h>
#include <algorithm>
#include <ostream>
#include <iostream>

using namespace std;

float dist(ClPoint p1, ClPoint p2) {
    return pow(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2), 0.5);
}

bool fits_cluster(vector<ClPoint> cluster, ClPoint point, float d) {
    for (int i = 0; i < cluster.size(); i++) {
        if (dist(point, cluster[i]) < d) {
            return true;
        }
    }
    return false;
}

vector<vector<ClPoint>> cluster(vector<ClPoint> points, float d) {
    vector<vector<ClPoint>> clusters{};
    
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

void test_clustering() {
    vector<ClPoint> points{ClPoint{.1, .1}, ClPoint{.1, .2}, ClPoint{.2, .3}, ClPoint{0, 1}, ClPoint{0, .9}, ClPoint{1, 1}};
    vector<vector<ClPoint>> expected{{ClPoint{1, 1}}, {ClPoint{0, .9}, ClPoint{0, 1}}, {ClPoint{.1, .1}, ClPoint{.1, .2}, ClPoint{.2, .3}}};
    vector<vector<ClPoint>> recieved = cluster(points, 0.2);

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
