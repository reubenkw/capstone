import matplotlib.pyplot as plt
import cv2
import math
import numpy as np
import time
from clustering import cluster
from plotting import plot_image

t_start = time.time_ns()

img_shape = (756, 800)
image = cv2.imread('scripts/data/flowers.jpg')
image = cv2.resize(image, img_shape)
window_name = 'flowers'

def detect_color(image: np.ndarray, color: np.ndarray, th_mag: float, th_ang: float) -> np.ndarray:
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
   
   dp = np.tensordot(image, color, axes=(2,0))

   # avoid divide by zero error
   img_mags[img_mags == 0] = near_zero
   if not col_mag:
      col_mag = near_zero
   
   mask = np.arccos(dp / col_mag / img_mags) < th_ang
   mask *= img_mags > th_mag
   return mask
   
def nearWhite(image, centroid, area):
   numWhitePixels=0
   length = int(math.sqrt(area))
   for i in range(-1*length, length):
      for j in range(-1*length, length):
         if (i < 0):
            x_index = min(max(0, int(centroid[1]) + i - length), len(image)-1)
         else :
            x_index = min(max(0, int(centroid[1]) + i + length), len(image)-1)
         if (j < 0):   
            y_index = min(max(0, int(centroid[0]) + j - length), len(image[0])-1)
         else :
            y_index = min(max(0, int(centroid[0]) + j + length), len(image[0])-1)
         if (np.all(image[x_index][y_index] == 255)):
            numWhitePixels+=1
   return numWhitePixels > area * 0.1

   
filtered = np.zeros_like(image)
# yellow = np.zeros([image.shape[0], image.shape[1]], np.uint8)
# for i in range (image.shape[0]-1) :
#   for j in range (image.shape[1]-1) :
#     if (detect_white([image[i][j][2], image[i][j][1], image[i][j][0]])):
#       filtered[i][j] = [255, 255, 255]
#     elif (detect_yellow([image[i][j][2], image[i][j][1], image[i][j][0]])):
#       filtered[i][j] = [0, 255, 255]
#       yellow[i][j] = 255

rgb = image[...,::-1].copy()
plot_image(rgb, filename="scripts/plots/original.png")
# ideal yellow in rgb
ideal_yellow = np.array([255, 255, 0])
filtered_yellow = detect_color(rgb, ideal_yellow, 200, 0.25)
yellow = filtered_yellow * 255
yellow = yellow.astype(np.uint8)

ideal_white = np.array([255, 255, 255])
filtered_white = detect_color(rgb, ideal_white, 175, 0.1)

ideal_yellow_bgr = ideal_yellow[::-1]
ideal_yellow_bgr_img = np.tile(ideal_yellow_bgr, (img_shape[1], img_shape[0], 1))
filtered = np.where(np.tile(filtered_yellow.T, (3, 1, 1)).T, ideal_yellow_bgr_img, filtered)

ideal_white_bgr = ideal_white[::-1]
ideal_white_bgr_img = np.tile(ideal_white_bgr, (img_shape[1], img_shape[0], 1))
filtered = np.where(np.tile(filtered_white.T, (3, 1, 1)).T, ideal_white_bgr_img, filtered)

filtered = filtered.astype(np.uint8)

# # Setup SimpleBlobDetector parameters.
# params = cv2.SimpleBlobDetector_Params()
 
# # Filter by Area.
# params.filterByArea = True
# params.minArea = 1
# params.maxArea = 100000

# # Filter by Circularity
# params.filterByCircularity = False
 
# # Filter by Convexity
# params.filterByConvexity = False
 
# # Filter by Inertia
# params.filterByInertia = False

# detector = cv2.SimpleBlobDetector_create(params)
# yellow_keypoints = detector.detect(yellow)
# print(len(yellow_keypoints))

# im_with_keypoints = cv2.drawKeypoints(yellow, yellow_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
output =  cv2.connectedComponentsWithStats(yellow, 8)

# Get the results
# The first cell is the number of labels
num_labels = output[0]
# The second cell is the label matrix
labels = output[1]
# The third cell is the stat matrix
stats = output[2]
# The fourth cell is the centroid matrix
centroids: np.ndarray = output[3]

plot_image(filtered[..., ::-1], filename="scripts/plots/masks.png")
print(f"Number of yellow centroids: {len(centroids)}")

filtered_centroids = []
for i in range(len(stats)-1):
   if (nearWhite(filtered, centroids[i], stats[i][cv2.CC_STAT_AREA])):
      filtered[int(centroids[i][1])][int(centroids[i][0])] = [0, 0, 255]
      filtered_centroids.append([centroids[i, 0], centroids[i, 1]])

plot_image(rgb, filtered_centroids, filename="scripts/plots/centroids.png")

# clusters are grouped together in lists
clusters = cluster(filtered_centroids, d = 50)
print(f"Number of clusters {len(clusters)}")
detections = []
for c in clusters:
   c_arr = np.array(c)
   c_avg = np.mean(c_arr, axis=0)
   detections.append((c_avg[0], c_avg[1]))
plot_image(rgb, detections, filename="scripts/plots/detected_flowers.png")

print(f"Time taken: {(time.time_ns() - t_start) * 1e-9:.5f} [s]")
# elin original commit: [37.40255, 25.84262]
