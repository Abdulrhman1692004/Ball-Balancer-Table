import cv2 as cv
import numpy as np
from sort import Sort
from collections import deque
import serial
import keyboard

#reciving and sending from uart variables:
COM = 'COM8'
baudrate = 115200
user_input = False

#Finding the setpoints variables:
x_setpoint = 8
y_setpoint = 8

#video capture from camera setup: connecting to ip_webcam and knowing the width and height of the image
video = cv.VideoCapture(0) #set it 0/1 when using the usb camera (DoriodCam app)
url = "http://192.168.1.5:8080/video" # Replace with your IP webcam URL you find in the app
video.open(url)



#PID:
kp = 0.0
ki = 0.0
kd = 0.0

#Recive from user variables:
def recive_from_user(x_setpoint, y_setpoint,kp,ki,kd,user_input):
    if keyboard.is_pressed("space") and not user_input:
        user_input = True
    
    if user_input:
        x,y = input("Enter coordinates (x,y,kp,ki,kd): ").split(',')
        print(f"Coordinates received: x={x}, y={y}")
        user_input = False
        x_setpoint = int(x)
        y_setpoint = int(y)
        kp = float(kp)
        ki = float(ki)  
        kd = float(kd)
        return x_setpoint, y_setpoint,kp,ki,kd,user_input
    else:
        pass
#computer vision class:
class ComputerVision:
    def __init__(self, video_source):
        self.mot_tracker = Sort(max_age=50, min_hits=1, iou_threshold=0.3)
        self.x = None
        self.y = None
        self.x_init = None
        self.y_init = None
        self.x_2_init = None
        self.y_2_init = None
        self.pixels_per_cm_x =None
        self.pixels_per_cm_y = None
        self.history_x = deque(maxlen=5)
        self.history_y = deque(maxlen=5)
        self.circularity = None
        self.circle_info = None  # (cx, cy, radius)
    
    def find_table(self, frame): #returning the roi: Which we need to return in a circle instead of a rectangle

        x1, y1 = 195, 70
        x2, y2 = 450, 327   
        #.pixels_per_cm_x = (x2 - x1) / 16 if (x2 - x1) > 0 else 1
        #self.pixels_per_cm_y = (y2 - y1) / 16 if    (y2 - y1) > 0 else 1   
        self.x_init, self.y_init = x1, y1
        self.x_2_init, self.y_2_init = x2, y2

        cx = (x1 + x2)//2
        cy = (y1 + y2)//2
        radius = (x2 - x1)//2
        self.circle_info = (cx, cy, radius)

        self.pixels_per_cm_x = (2*radius) / 16
        self.pixels_per_cm_y = (2*radius) / 16

        cv.circle(frame, (cx,cy), radius, (0,0,0),2)
        cv.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 2)

        roi = frame[y1:y2, x1:x2]

        # Create circular mask
        mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        cv.circle(mask, (roi.shape[1] // 2, roi.shape[0] // 2), radius, 255, -1)

        # Apply mask
        masked_roi = cv.bitwise_and(roi, roi, mask=mask)

        cv.putText(frame, f".", (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(frame, f'.', (x2, y2), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return masked_roi
        
        
    def filter_image(self, frame):
    
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (9, 9), 0)
        return blurred
    
    def detect_edges(self, frame):
            edges = cv.Canny(frame, 100, 200)
            return edges

        

    def detect_ball(self, frame):
        contours, _ = cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        detections = []
        roi_h, roi_w = frame.shape[:2]
        cx_roi = roi_w // 2
        cy_roi = roi_h // 2
        _, _, radius_roi = self.circle_info

        for cnt in contours:
            area = cv.contourArea(cnt)
            frame_area = frame.shape[0] * frame.shape[1]
            rltv_area = area / frame_area
            perimeter = cv.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))


            if rltv_area > 0.01 and rltv_area < 0.8 and circularity > 0.7:  # filter out small noise
                (x, y), radius = cv.minEnclosingCircle(cnt)
                x , y , radius = int(x), int(y), int(radius)

                if (x - cx_roi) ** 2 + (y - cy_roi) ** 2 > radius_roi ** 2:
                    continue

                detections.append([x - radius, y - radius, x + radius, y + radius, circularity])
                self.circularity = circularity
                   
        return np.array(detections) if detections else np.empty((0, 5))

    def track_ball(self, frame, detections): #now instead of processing the whole frame, just process the ROI, show it and return its values
        tracker_results = self.mot_tracker.update(detections)
        for tracked in tracker_results:
            x1, y1, x2, y2, ID = map(int, tracked)
            x1 += self.x_init
            x2 += self.x_init
            y1 += self.y_init
            y2 += self.y_init
            cx = (x1 + x2) / 2
            cy = (y1 + y2) // 2
            


            self.history_x.append(cx)
            self.history_y.append(cy)
            self.x = np.mean(self.history_x)
            self.y = np.mean(self.history_y) 

            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(frame, f'x: {int((cx - self.x_init)/self.pixels_per_cm_x)} y: {int((cy-self.y_init)/self.pixels_per_cm_y)}, circularity:{self.circularity}', 
                       (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
class uart_handler:
    def __init__(self, COM, baudrate, time_out):
        self.com = COM
        self.baudrate = baudrate
        self.timeout = time_out
        self.uart = serial.Serial(self.com, self.baudrate, timeout=self.timeout)
       

    def send_message(self, msg):
      if self.uart.is_open:
            self.uart.write(msg.encode('utf-8'))    
    def stop(self):
        """Stop the UART thread and close the port."""
        if self.uart.is_open:
            self.uart.close()

ball_finder = ComputerVision (video)
uarter = uart_handler(COM, baudrate, 1)
try:
    while True:
        for _ in range(3):
            video.grab()
        

        ret, frame = video.retrieve()

        #frame = cv.flip(frame, -1)
        height, width= frame.shape[:2]
        frame_area = height * width


        x_setpoint, y_setpoint,kp,ki,kd,user_input = recive_from_user(x_setpoint, y_setpoint,kp,ki,kd,user_input)
        ROI = ball_finder.find_table(frame)
        filtered = ball_finder.filter_image(ROI)
        edges = ball_finder.detect_edges(filtered)
        detections = ball_finder.detect_ball(edges)
        ball_finder.track_ball(frame, detections)
   

        cv.imshow("Ball Detection", frame)

        

        if ball_finder.x is not None and ball_finder.y is not None:
            x = int(( ball_finder.x - ball_finder.x_init)/ball_finder.pixels_per_cm_x)
            y = int(( ball_finder.y - ball_finder.y_init)/ball_finder.pixels_per_cm_y)
            msg_to_print = f"x: {x}, x_pixel: {( ball_finder.x - ball_finder.x_init)},  y: {y}, y_pixel:{( ball_finder.y - ball_finder.y_init)} \n"
            print(msg_to_print)
            msg = f"{x_setpoint},{y_setpoint},{x},{y},{kp},{ki},{kd}\n"
            uarter.send_message(msg)
        if cv.waitKey(1) & 0xFF == 27:
            break
finally:
    video.release()
    cv.destroyAllWindows()
    uarter.stop()
