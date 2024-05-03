import RPi.GPIO as gpio
import time
import cv2
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

printed = False

count = 0

print("Started serial thingies")
while not printed:
    if ser.in_waiting > 0:
        print(ser.readline())