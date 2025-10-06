#the code to identify b, g ,r
import cv2

# Load image
image = cv2.imread("image.jpg")

# Coordinates of the pixel you want to check
x, y = 100, 200

# OpenCV reads in BGR format
(b, g, r) = image[y, x]
print(f"BGR value at ({x}, {y}): B={b}, G={g}, R={r}")
