# orange_detector.py

import cv2
import numpy as np

# Define orange color range in HSV
LOWER_ORANGE = np.array([10, 100, 100])
UPPER_ORANGE = np.array([25, 255, 255])

def detect_orange_objects(image_bgr):
    """
    Detect orange objects in an image using HSV color masking.
    
    Args:
        image_bgr: OpenCV BGR image (numpy array)

    Returns:
        image_bgr: Image with bounding boxes drawn
        boxes: List of (x, y, w, h) for each detected object
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE) #Create a binary mask where all pixels within a specified HSV range (here: LOWER_ORANGE to UPPER_ORANGE) are white (255), and all others are black (0).
    mask = cv2.erode(mask, None, iterations=1) #Perform erosion, which removes small white noise (tiny specks or dots).
    mask = cv2.dilate(mask, None, iterations=2) #Grows white regions, filling in gaps or holes.

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []

    for cnt in contours:
    
        area = cv2.contourArea(cnt)
        if area > 2000:
            x, y, w, h = cv2.boundingRect(cnt)
            boxes.append((x, y, w, h))
            cv2.rectangle(image_bgr, (x, y), (x + w, y + h), (0, 140, 255), 2)
            cv2.putText(image_bgr, "Orange Object", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            print(f"[orange_detector] Detected orange object at (x={x}, y={y}, w={w}, h={h}) with area={area}")

    return image_bgr, boxes, mask
