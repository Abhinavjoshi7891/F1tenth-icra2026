import cv2
import numpy as np
import os

def clean_map_advanced(map_name):
    map_dir = f'/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/{map_name}'
    input_path = os.path.join(map_dir, f'{map_name}.png')
    output_path = os.path.join(map_dir, f'{map_name}.png')
    pf_output_path = os.path.join(map_dir, 'pf_map.png')

    # Load image
    img = cv2.imread(input_path, 0)
    if img is None:
        print(f"Error: Could not load image at {input_path}")
        return

    # 1. Binarize (ensure only 0 and 255)
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # 2. Heal the shape (Closing)
    # Using a 7x7 kernel to bridge gaps and fill small holes inside the track
    kernel_heal = np.ones((7,7), np.uint8)
    healed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_heal)

    # 3. Remove "Spurs" and Noise (Opening)
    # This removes the tiny one-pixel jagged edges that cause the skeleton "dots"
    kernel_smooth = np.ones((3,3), np.uint8)
    smoothed = cv2.morphologyEx(healed, cv2.MORPH_OPEN, kernel_smooth)

    # 4. Blur and Final Threshold (The "Melt" effect)
    # This rounds out the corners so the skeleton is a smooth curve
    blurred = cv2.GaussianBlur(smoothed, (5,5), 0)
    _, final = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    # 5. Save
    cv2.imwrite(output_path, final)
    cv2.imwrite(pf_output_path, final)
    print(f"Successfully applied ADVANCED cleaning to {map_name}. Centerline should be much smoother now.")

if __name__ == "__main__":
    clean_map_advanced('home_v4')
