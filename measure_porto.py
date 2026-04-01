import cv2
import numpy as np
import os

map_path = '/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/porto.png'
resolution = 0.05

if not os.path.exists(map_path):
    print(f"Error: {map_path} not found.")
    exit(1)

img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# Distance transform gives distance to nearest zero (black) pixel
dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)

# Calculate widths in meters
max_dist = np.max(dist)
# The centerline value would be the peak of the distance transform
# Width across the track at that point is roughly 2 * max_dist
max_width_m = 2 * max_dist * resolution

# We also want the "minimum" width along the main track
# To do this, we can mask the distance transform with the skeleton
from skimage.morphology import skeletonize
from skimage import img_as_bool

skeleton = skeletonize(img_as_bool(binary))
skeleton_dist = dist[skeleton]

min_width_m = 2 * np.min(skeleton_dist) * resolution
avg_width_m = 2 * np.mean(skeleton_dist) * resolution

print(f"Image Dimensions: {img.shape[1]}x{img.shape[0]}")
print(f"Max Track Width (m): {max_width_m:.3f}")
print(f"Min Track Width (m): {min_width_m:.3f}")
print(f"Avg Track Width (m): {avg_width_m:.3f}")
print(f"Safety Width Param (m): 0.7")
