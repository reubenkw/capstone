from typing import Optional
import random
from matplotlib import pyplot as plt
import numpy as np


T_point = tuple[float, float]


def dist(p1, p2) -> float:
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def fits_cluster(cluster: list[T_point], point: T_point, d: float) -> bool:
    for p_compare in cluster:
        if dist(point, p_compare) < d:
            return True
    return False


def cluster(points: list[T_point], d: float) -> list[list[T_point]]:
    clusters: list[Optional[list[T_point]]] = []

    for point in points:
        fits_clusters = [
            i for i, cluster in enumerate(clusters) if fits_cluster(cluster, point, d)
        ]
        if not fits_clusters:
            clusters.append([point])
            continue
        prnt_cluster = fits_clusters.pop(0)
        clusters[prnt_cluster].append(point)

        # combine all remaining matched clusters
        for i in reversed(fits_clusters):
            clusters[prnt_cluster].extend(clusters.pop(i))

    return clusters


def centroids_from_clusters(clusters: list[list[T_point]]) -> list[T_point]:
    detections = []
    for c in clusters:
        c_arr = np.array(c)
        c_avg = np.mean(c_arr, axis=0)
        detections.append((c_avg[0], c_avg[1]))

    return detections


def cluster_optimize(points: list[T_point], d: float) -> list[list[T_point]]:
    pass


def test():
    n_points = 200
    dist = 0.05
    rand_points = [(random.random(), random.random()) for _ in range(n_points)]

    clustered_points = cluster(rand_points, dist)
    print(rand_points)
    print("___")
    print(clustered_points)
    print(f"n clusters {len(clustered_points)}")

    for i, cluster1 in enumerate(clustered_points):
        x, y = zip(*cluster1)
        plt.scatter(x, y)

        for n in range(len(x)):
            plt.annotate(i, (x[n], y[n]))

    plt.show()


if __name__ == "__main__":
    test()
