import RPi.GPIO as gpio
import time
import serial


ser = serial.Serial('/dev/ttyUSB0', 9600)

printed = False

count = 0

while not printed:
    if ser.in_waiting > 0:
        count += 1
        ser.readline()

        if count > 10:
            data = ser.readline()
            print(data)

            # data = float(str(data.rstrip().lstrip()).strip("'").strip("b'")[2:7])
            # print(data)

            printed = True

angle = float(data.split(" ")[1][:-4])
print(angle)