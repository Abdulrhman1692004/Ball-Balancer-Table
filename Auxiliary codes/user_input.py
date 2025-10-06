import keyboard
import time
user_input = False
x_setpoint = None
y_setpoint = None

def recive_from_user():
    global user_input
    if keyboard.is_pressed("space") and not user_input:
        user_input = True
    
    if user_input:
        x,y = input("Enter coordinates (x,y): ").split(',')
        print(f"Coordinates received: x={x}, y={y}")
        user_input = False
        x_setpoint = int(x)
        y_setpoint = int(y)


while True:
        recive_from_user()
        time.sleep(0.1)
        




