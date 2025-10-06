import cv2 as cv
import numpy as np

def detect_shapes(frame):
    # Resize for consistency
    frame = cv.resize(frame, (600, 400))

    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Blur to reduce noise
    blurred = cv.GaussianBlur(gray, (7, 7), 0)

    # Edge detection
    edges = cv.Canny(blurred, 50, 150)

    # Morphological closing + dilation to connect edges
    kernel = np.ones((5, 5), np.uint8)
    edges = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < 1000:  # filter out small junk edges
            continue

        # Approximate contour to polygon
        epsilon = 0.02 * cv.arcLength(cnt, True)  # 2% of perimeter
        approx = cv.approxPolyDP(cnt, epsilon, True)

        if len(approx) >= 4:
            # It's a quadrilateral, could be square or rectangle
            # Get bounding rect for drawing
            x, y, w, h = cv.boundingRect(approx)

            # Draw the rectangle
            cv.drawContours(frame, [approx], 0, (0, 255, 0), 3)
            cv.putText(frame, "circle", (x, y - 10), cv.FONT_HERSHEY_SIMPLEX,
                       0.7, (255, 0, 0), 2)

    return frame, edges


# --- MAIN VIDEO LOOP ---
VIDEO_ADDRESS = "http://10.104.173.38:8080/video"  # Example for IP Webcam app
cap = cv.VideoCapture(VIDEO_ADDRESS)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break  # No frame

    detected_frame, edges = detect_shapes(frame)

    # Show results
    cv.imshow("Edges", edges)
    cv.imshow("Shape Detection", detected_frame)

    # Exit when 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
