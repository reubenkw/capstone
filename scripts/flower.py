import matplotlib.pyplot as plt
import cv2
import math
import np

image = cv2.imread('flowers.jpg')
image = cv2.resize(image, (756, 800))
window_name = 'flowers'
# cv2.imshow(window_name, image)
# cv2.waitKey(0) 
# cv2.destroyAllWindows() 

def detect_white(color):
   if (mag(color) > 175):
      ideal_white = [255, 255, 255]
      dp = dot_product(color, ideal_white )
      return math.acos(dp / mag(color) / mag(ideal_white)) < 0.1
   else : 
      return False

def dot_product(x, y):
   return x[0] * y[0] + x[1] * y[1] + x[2] * y[2]

def mag(x):
   return math.sqrt(x[0]**2 + x[1]**2 + x[2]**2)
   
def detect_yellow(color):
   if (mag(color) > 200):
      ideal_yellow = [255, 255, 0]
      dp = dot_product(color, ideal_yellow )
      return math.acos(dp / mag(color) / mag(ideal_yellow)) < 0.25
   else :
      return False
   
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
yellow = np.zeros([image.shape[0], image.shape[1]], np.uint8)
for i in range (image.shape[0]-1) :
  for j in range (image.shape[1]-1) :
    if (detect_white([image[i][j][2], image[i][j][1], image[i][j][0]])):
      filtered[i][j] = [255, 255, 255]
    elif (detect_yellow([image[i][j][2], image[i][j][1], image[i][j][0]])):
      filtered[i][j] = [0, 255, 255]
      yellow[i][j] = 255

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
centroids = output[3]

print(len(centroids))
print(len(filtered))
print(len(filtered[0]))

numBlobs = 0

for i in range(len(stats)-1):
   if (nearWhite(filtered, centroids[i], stats[i][cv2.CC_STAT_AREA])):
      filtered[int(centroids[i][1])][int(centroids[i][0])] = [0, 0, 255]
      numBlobs += 1

print(numBlobs)
      
 
cv2.imshow(window_name, filtered)
cv2.waitKey(0) 
cv2.destroyAllWindows() 


