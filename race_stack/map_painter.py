import cv2
import numpy as np
import sys
import os

def run_painter(target_path):
    BRUSH_SIZE = 0  # 0 radius = 1 pixel thick brush
    
    # Load skeleton
    img = cv2.imread(target_path, 0)
    if img is None:
        print(f"Error: Could not load {target_path}")
        return
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # Load original map to show as background
    map_dir = os.path.dirname(target_path)
    map_name = os.path.basename(map_dir)
    map_path = os.path.join(map_dir, f"{map_name}.png")
    
    if os.path.exists(map_path):
        bg_img = cv2.imread(map_path, 0)
        # Flip map vertically to match how the node loaded the skeleton
        bg_img = cv2.flip(bg_img, 0)
        _, bg_img = cv2.threshold(bg_img, 200, 255, cv2.THRESH_BINARY)
        # Dim the background map (free space = 100, walls = 0)
        bg_img = (bg_img // 255) * 60
    else:
        bg_img = np.zeros_like(img)

    # Scale up for easier editing
    SCALE = 4
    
    def update_display():
        # Overlay bright white skeleton on dim map
        combined = np.maximum(bg_img.astype(np.uint8), img)
        return cv2.resize(combined, (img.shape[1]*SCALE, img.shape[0]*SCALE), interpolation=cv2.INTER_NEAREST)
        
    display = update_display()
    
    def paint(event, x, y, flags, param):
        nonlocal display, BRUSH_SIZE
        # Scale back to original coordinates
        ox, oy = x // SCALE, y // SCALE

        # Left click OR Left-drag
        if event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON)):
            # Check if SHIFT key is held down
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                color = 255 # White (Draw)
            else:
                color = 0   # Black (Erase)
                
            cv2.circle(img, (ox, oy), BRUSH_SIZE, color, -1)
            display = update_display()

    cv2.namedWindow('Skeleton Painter')
    cv2.setMouseCallback('Skeleton Painter', paint)

    print(f"=== SKELETON PAINTER: Editing {os.path.basename(target_path)} ===")
    print("LEFT CLICK+DRAG        → Erase Horn (Black)")
    print("SHIFT + LEFT CLICK     → Connect Gap (White)")
    print("Press 'S'              → Save and quit")
    print("Press 'Q'              → Quit without saving")
    print("Press '+' / '-'        → Adjust Brush Size")

    while True:
        cv2.imshow('Skeleton Painter', display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            cv2.imwrite(target_path, img)
            print("Saved skeleton!")
            break
        elif key == ord('q'):
            print("Quit without saving.")
            break
        elif key == ord('+') or key == ord('='):
            BRUSH_SIZE += 1
            print(f"Brush size radius: {BRUSH_SIZE}")
        elif key == ord('-'):
            BRUSH_SIZE = max(0, BRUSH_SIZE - 1)
            print(f"Brush size radius: {BRUSH_SIZE}")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 map_painter.py <path_to_image>")
    else:
        run_painter(sys.argv[1])
