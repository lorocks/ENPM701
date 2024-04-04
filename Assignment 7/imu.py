import RPi.GPIO as gpio
import time
import serial


ser = serial.Serial('/dev/ttyUSB0', 9600)

printed = False

while not printed:
    if ser.in_waiting > 0:
        data = ser.readline()
        print(data)

        data = float(str(data.rstrip().lstrip()).strip("'").strip("b'")[2:7])
        print(data)

        printed = True