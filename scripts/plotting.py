import matplotlib.pyplot as plt
import numpy as np

def plot_image(img: np.ndarray, points: list = [], filename="plot.png") -> None:
    plt.figure()
    if img.ndim == 2:
        # grayscale
        plt.imshow(img, cmap="gray")
    else:
        plt.imshow(img)
    if points:
        plt.scatter(*zip(*points))

    plt.tight_layout()
    plt.savefig("scripts/plots/" + filename)

