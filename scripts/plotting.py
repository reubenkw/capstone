import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import pickle
import io


def plot_save_image(img: np.ndarray, points: list = [], point_labels: list = [], filename="plot.png"):
    plt.figure()
    if img.ndim == 2:
        # grayscale
        plt.imshow(img, cmap="gray")
    else:
        plt.imshow(img)
    if points:
        plt.scatter(*zip(*points))
    if point_labels:
        assert len(points) == len(point_labels), "# point labels does not match # points"
        for i in range(len(points)):
            plt.annotate(point_labels[i], points[i])

    plt.tight_layout()
    plt.savefig("scripts/plots/" + filename)


def plot_image(ax, img: np.ndarray, points: list = [], point_labels: list = [], title=""):
    if img.ndim == 2:
        # grayscale
        ax.imshow(img, cmap="gray")
    else:
        ax.imshow(img)
    if points:
        ax.scatter(*zip(*points))
    if point_labels:
        assert len(points) == len(point_labels), "# point labels does not match # points"
        for i in range(len(points)):
            ax.annotate(point_labels[i], points[i])

    ax.set_title(title)
