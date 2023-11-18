import matplotlib.pyplot as plt
import scipy.signal as signal
import cv2
import math
import numpy as np
import time
from clustering import cluster, centroids_from_clusters
from plotting import plot_image
from utils import timing


@timing(print_args=False)
def detect_color(
    image: np.ndarray, color: np.ndarray, th_mag: float, th_ang: float
) -> np.ndarray:
    """
    image: image with shape (X, Y, 3)
    color: color in same space as image (3,)
    th_mag: each pixel in the image must have at least this magnitude
    th_ang: threshold for angle between each pixel and filter color
    returns:
       mask: shape (X, Y)
    """
    near_zero = 0.0001

    col_mag = np.linalg.norm(color)
    img_mags = np.linalg.norm(image, axis=2)

    dp = np.tensordot(image, color, axes=(2, 0))

    # avoid divide by zero error
    img_mags[img_mags == 0] = near_zero
    if not col_mag:
        col_mag = near_zero

    mask = np.arccos(dp / col_mag / img_mags) < th_ang
    mask *= img_mags > th_mag
    return mask


def nearWhite(img, centroid) -> bool:
    return img[int(centroid[1])][int(centroid[0])] > 100


@timing(print_args=False)
def blur_img(img, numRows=100, stdDev=50):
    oneDimGaussian = np.outer(signal.windows.gaussian(numRows, stdDev), 1)
    oneDimGaussian = oneDimGaussian / np.sum(oneDimGaussian)
    blur = signal.convolve2d(img, oneDimGaussian)
    blur = signal.convolve2d(blur, np.transpose(oneDimGaussian))
    pad = int(numRows / 2)

    blur = blur[pad : len(blur) - pad + 1]
    blur = blur[:, pad : len(blur[0]) - pad + 1]
    blur /= np.max(blur)
    blur *= 255.0

    return blur


@timing(print_args=False)
def add_mask_to_img(img, mask, color) -> np.ndarray:
    """
    img: rgb (width, height, 3)
    mask: greyscale (width, height)
    color: rgb color (3,) or image (width, height)
    """
    width, height, _ = img.shape
    if color.ndim == 1:
        color = np.tile(color, (width, height, 1))
    return np.where(np.tile(mask.T, (3, 1, 1)).T, color, img).astype(np.uint8)


# TODO: switch to float coordinates 0.0->1.0
@timing(print_args=False)
def detect_flowers(img) -> tuple[list[tuple[int, int]], dict]:
    """
    img: rgb
    return:
       detections: list of x, y points in pixels from top left
       info: dict
    """
    info = {}

    # colors in rgb
    ideal_yellow = np.array([255, 255, 0])
    ideal_white = np.array([255, 255, 255])

    mask_yellow = detect_color(img, ideal_yellow, 200, 0.25)
    info["mask_yellow"] = mask_yellow
    mask_white = detect_color(img, ideal_white, 175, 0.1)

    blurred_white = blur_img(mask_white)
    info["blur_white"] = blurred_white

    mask_overlay = np.zeros_like(img)
    mask_overlay = add_mask_to_img(mask_overlay, mask_yellow, ideal_yellow)
    mask_overlay = add_mask_to_img(mask_overlay, mask_white, ideal_white)
    info["masks"] = mask_overlay

    output = cv2.connectedComponentsWithStats((mask_yellow * 255).astype(np.uint8), 8)
    # The third cell is the stat matrix
    # stats = output[2]
    # The fourth cell is the centroid matrix
    centroids: np.ndarray = output[3]
    centroids = centroids.tolist()
    info["n_centroids"] = len(centroids)
    info["centroids"] = centroids

    filtered_centroids = []
    for c in centroids:
        if nearWhite(blurred_white, c):
            filtered_centroids.append([c[0], c[1]])

    # clusters are grouped together in lists
    clusters = cluster(filtered_centroids, d=50)
    detections = centroids_from_clusters(clusters)
    info["n_detections"] = len(detections)

    return detections, info


def test_detect_flower(path="scripts/data/flowers.jpg"):
    img_shape = (756, 800)
    # original in rgb
    original = cv2.imread(path)[..., ::-1]
    original = cv2.resize(original, img_shape)
    plot_image(original, filename="original.png")

    detections, info = detect_flowers(original)
    plot_image(original, info["centroids"], filename="centroids.png")
    plot_image(original, detections, filename="detected_flowers.png")
    plot_image(info["masks"], filename="masks.png")
    plot_image(info["blur_white"], filename="blurred_white.png")


if __name__ == "__main__":
    test_detect_flower()
