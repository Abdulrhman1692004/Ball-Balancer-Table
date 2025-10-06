import serial
import time

uart = serial.Serial('COM8', 115200 , timeout=1)

while True:
        uart.write("s:0\n".encode('utf-8'))
        print("s:0")
        time.sleep(2)
        uart.write("s:90\n".encode('utf-8'))
        print("s:90")
        time.sleep(2)
        uart.write("s:180\n".encode('utf-8'))
        print("s:180")
        time.sleep(2)
 




