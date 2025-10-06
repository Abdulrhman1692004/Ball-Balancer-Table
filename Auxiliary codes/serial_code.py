import serial
import random

uart = serial.Serial('COM8', 115200 )

while True: 
    x_setpoint = random.randint(0,15)
    y_setpoint = random.randint(0,15)
    x = random.randint(0,15)
    y = random.randint(0,15)
    msg = f"{x_setpoint},{y_setpoint},{x},{y}\n"
    uart.write(msg.encode('utf-8'))
    print(msg)