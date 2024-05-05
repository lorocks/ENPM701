import RPi.GPIO as gpio
import time
import cv2
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

printed = False

count = 0

print("Started serial thingies")

ser.reset_input_buffer()
time.sleep(0.1)

while not printed:
    if ser.in_waiting > 0:
        count += 1
        print(ser.readline())

        if count > 10:
            printed = True
print("Serial cleaned")
