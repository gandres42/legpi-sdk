import cv2
import numpy as np
import os

SCAN_COLORS = ['blue', 'red', 'green']

# Start video capture
cap = cv2.VideoCapture(0)

def find_centers(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = []
    if 'red' == color:
        lower_bound = np.array([0, 120, 70])
        upper_bound = np.array([10, 255, 255])
        masks.append(cv2.inRange(hsv, lower_bound, upper_bound))
    if 'blue' == color:
        lower_bound = np.array([100, 150, 50])
        upper_bound = np.array([140, 255, 255])
        masks.append(cv2.inRange(hsv, lower_bound, upper_bound))
    if 'green' == color:
        lower_bound = np.array([40, 50, 50])
        upper_bound = np.array([90, 255, 255])
        masks.append(cv2.inRange(hsv, lower_bound, upper_bound))
    mask = sum(masks)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centers = []
    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # Ignore small objects
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # print(f"Object center: ({cx}, {cy})"                
                centers.append((cx, cy))
    return centers

if __name__ == "__main__":
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        centers = []
        for color in SCAN_COLORS:
            centers = centers + find_centers(frame, color)

        for cx, cy in centers:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)  

        # Show the processed frame
        cv2.imshow("Red Object Detection", frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
