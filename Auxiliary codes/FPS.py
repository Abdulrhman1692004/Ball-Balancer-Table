import cv2 as cv
import time

# --- Camera setup ---
video = cv.VideoCapture(1)

if not video.isOpened():
    print("❌ Failed to open video stream.")
    exit()

# --- FPS Measurement Variables ---
frame_count = 0
start_time = time.time()
fps = 0.0  # Initialize to avoid "not defined" error

while True:
    ret, frame = video.read()
    if not ret:
        print("⚠️ Failed to grab frame.")
        break

    frame_count += 1
    elapsed_time = time.time() - start_time

    # Calculate FPS every second
    if elapsed_time >= 1.0:
        fps = frame_count / elapsed_time
        frame_count = 0
        start_time = time.time()
        print(f"FPS: {fps:.2f}")

    # Display FPS on the frame
    cv.putText(frame, f"FPS: {fps:.2f}", (10, 40),
               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv.imshow("FPS Measurement", frame)

    # Exit with ESC key
    if cv.waitKey(1) & 0xFF == 27:
        break

video.release()
cv.destroyAllWindows()
