# clean_advanced.py

import cv2
import numpy as np
import os

# Paths
map_path = '/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/porto.png'
clean_path = '/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/porto_clean.png'
pf_path = '/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/pf_map.png'

# Read the image
img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# 1. Fill holes (removes the center line ghosting)
# We use a large kernel to bridge the gap of that internal black line
kernel = np.ones((21,21), np.uint8)
closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

# 2. Smooth the edges
# Smoothing melts away single-pixel noise that causes skeleton branching
blurred = cv2.GaussianBlur(closed, (9,9), 0)
_, final = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

# 3. Save to both filenames
cv2.imwrite(clean_path, final)
cv2.imwrite(pf_path, final)

print(f"Successfully processed {map_path}")
print(f"Saved clean version to {clean_path}")
print(f"Saved localization version to {pf_path}")
