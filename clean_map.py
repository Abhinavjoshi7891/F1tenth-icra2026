# clean_map.py

import cv2
import numpy as np
import os

# Paths
map_path = '/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/porto.png'
output_path = '/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/porto_clean.png'

# Check if file exists
if not os.path.exists(map_path):
    print(f"Error: {map_path} not found.")
    exit(1)

# Read the image
img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

# 1. Ensure binary (Threshold)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# 2. Smooth the edges
blurred = cv2.GaussianBlur(binary, (5, 5), 0)

# 3. Re-Threshold
_, cleaned = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

# 4. Morphological Opening
kernel = np.ones((3, 3), np.uint8)
opened = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel, iterations=1)

# Save the result to porto_clean.png
cv2.imwrite(output_path, opened)

print(f"Successfully saved clean map image to {output_path}")
