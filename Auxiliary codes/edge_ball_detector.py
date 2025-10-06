import cv2 as cv
import numpy as np

# Define the region of interest (ROI) as (x, y, width, height)

def detect_balls(frame):
    frame = cv.flip(frame, -1)

    # Resize for consistency

    # Draw the ROI rectangle on the frame
    x1 = 200
    y1 = 70 
    x2 = 440
    y2 = 318
    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)  # filled rectangle for text background

    # Extract ROI from the frame
    roi_frame = frame[y1:y2, x1:x2]

    # Convert to grayscale
    gray = cv.cvtColor(roi_frame, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (9, 9), 0)

    # Edge detection
    edges = cv.Canny(blurred, 140, 200)

    #
    # Find contours
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv.contourArea(cnt)
        frame_area = roi_frame.shape[0] * roi_frame.shape[1]
        rltv_area = area / frame_area
        perimeter = cv.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * (area / (perimeter * perimeter))

        if rltv_area > 0.01 and rltv_area < 0.8 and circularity > 0.5:  # filter out small noise
            ((x, y), radius) = cv.minEnclosingCircle(cnt)
            cv.circle(frame, (int(x) + x1, int(y) + y1), int(radius), (0, 255, 0), 2)
            cv.putText(frame, f'x: {int((x + x1))} y: {int((y + y1))}', (int(x) + x1 - 40, int(y) + y1 - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        # Draw circle (shift coordinates relative to full frame)IMPLEX, 0.6, (255, 0, 0), 2)
    return frame, edges


# --- MAIN VIDEO LOOP ---

cap = cv.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break  # No frame

    detected_frame, edges = detect_balls(frame)

    # Show results
    cv.imshow("Edges", edges)
    cv.imshow("Ball Detection", detected_frame)

    # Exit when 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
