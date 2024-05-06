import numpy as np
import math

# Video Streaming
import cv2
from threading import Thread

import matplotlib.pyplot as plt
import imutils
import time

# RPi Stuff
import RPi.GPIO as gpio
import os
from datetime import datetime

# Email sending
import base64
import smtplib
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

# IMU Serial
import serial


# Initialise Robot Params
wheel_radius = 1.3
encoder_tick = 8
motor_rots = 120


# Initialise Serial
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Initialise RPi Pins
trig = 16
echo = 18

servo = 36

open_s = 5
close = 7.9

pwm_val = 75
Kp = -2.5

gpio.setmode(gpio.BOARD)
gpio.setup(servo,gpio.OUT)

gpio.setup(31,gpio.OUT) #IN1
gpio.setup(33,gpio.OUT) #IN2
gpio.setup(35,gpio.OUT) #IN3
gpio.setup(37,gpio.OUT)

gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

gpio.output(trig, False)
time.sleep(0.01)

pwm_servo = gpio.PWM(servo,50)
pwm_servo.start(close)

pwm31 = gpio.PWM(31, 50)
pwm33 = gpio.PWM(33, 50)
pwm35 = gpio.PWM(35, 50)
pwm37 = gpio.PWM(37, 50)

gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)


def getangle():
        ser.reset_input_buffer()
        time.sleep(0.1)
        data = ser.readline()
        data = data.decode()
        try:
            angle = float(data.split(" ")[1][:-4])
        except:
            angle = -1
        
        return angle


def lefttill(angle):
        # ser.reset_input_buffer()
        # time.sleep(0.1)
        # data = ser.readline()
        # data = data.decode()
        # try:
        #     initial_angle = float(data.split(" ")[1][:-4])
        # except:
        #     initial_angle = 0
        

        gottem = False
        while not gottem:
            initial_angle = getangle()
            if initial_angle != -1:
                gottem = True   

        current_angle = initial_angle 
        
        pwm33.start(90)
        pwm37.start(90)

        while current_angle < angle - 4 or current_angle > angle + 4:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm33.stop()
        pwm37.stop()

        return current_angle


angle = lefttill(360 - 120)

print(angle)

for i in range(10):
    print(getangle())

pwm_servo.stop()
pwm31.stop()
pwm33.stop()
pwm35.stop()
pwm37.stop()
gpio.cleanup()