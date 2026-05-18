import cv2
import numpy as np
import os

def clean_map(map_name):
    map_dir = f'/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/{map_name}'
    input_path = os.path.join(map_dir, f'{map_name}.png')
    output_path = os.path.join(map_dir, f'{map_name}.png') # Overwrite with clean version
    pf_output_path = os.path.join(map_dir, 'pf_map.png')

    # Load image
    img = cv2.imread(input_path, 0)
    if img is None:
        print(f"Error: Could not load image at {input_path}")
        return

    # 1. Binarize
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # 2. Close Gaps (The "Heal" operation)
    # Using a 5x5 kernel to bridge the 1-2 pixel gap you have
    kernel = np.ones((5,5), np.uint8)
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 3. Smooth Edges
    blurred = cv2.GaussianBlur(closed, (3,3), 0)
    _, final = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    # 4. Save
    cv2.imwrite(output_path, final)
    cv2.imwrite(pf_output_path, final)
    print(f"Successfully cleaned {map_name}. Gap bridged and edges smoothed.")

if __name__ == "__main__":
    clean_map('home_v4')
