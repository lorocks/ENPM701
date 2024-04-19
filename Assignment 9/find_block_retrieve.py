import numpy as np
import math

import cv2
from threading import Thread

import matplotlib.pyplot as plt
import imutils
import time
import RPi.GPIO as gpio
import os
from datetime import datetime


import base64
import smtplib
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

import serial

# Initialise Robot Params
wheel_radius = 1.3
encoder_tick = 8
motor_rots = 120


# Initialise Serial
ser = serial.Serial('/dev/ttyUSB0',9600)
printed = False

count = 0

while not printed:
    if ser.in_waiting > 0:
        count += 1
        ser.readline()

        if count > 10:
            printed = True



# Initialise Video Stream
class VideoStream:
    """Camera object that controls video streaming from the Picamera"""

    def __init__(self, resolution=(640, 480), framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3, resolution[0])
        ret = self.stream.set(4, resolution[1])

        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

        # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
        # Start the thread that reads frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Return the most recent frame
        return self.frame

    def stop(self):
        # Indicate that the camera and thread should be stopped
        self.stopped = True

# Initialize the webcam
videostream = VideoStream().start()
time.sleep(1)


# Initialise Video Writer
frame_width = int(640)
frame_height = int(480)

size = (frame_width, frame_height)

result = cv2.VideoWriter('my_video.avi',
                        cv2.VideoWriter_fourcc(*'MJPG'),
                        5, size)


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

pwm_servo = gpio.PWM(servo,50)
pwm_servo.start(close)

pwm31 = gpio.PWM(31, 50)
pwm33 = gpio.PWM(33, 50)
pwm35 = gpio.PWM(35, 50)
pwm37 = gpio.PWM(37, 50)

gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)


# Initialise HSV Colors
lower_green = np.array([20, 10, 60])
upper_green = np.array([60, 255, 255])

lower_blue = np.array([110, 100, 80])
upper_blue = np.array([115, 255, 255])

lower_red = np.array([0, 100, 100])
upper_red = np.array([5, 255, 255])

lower = [lower_blue, lower_green, lower_red]
upper = [upper_blue, upper_green, upper_red]

block_color = int(input("Enter BGR -> 1 2 3")) - 1


try:

    def gameover():
        gpio.output(31,False)
        gpio.output(33,False)
        gpio.output(35,False)
        gpio.output(37,False)

    def timeskip(count):
        start = time.time()
        while time.time() - start < count:
            ser.readline()

        
    def forward(encoder_count):
        counter_r = counter_l = 0
        tick_r = tick_l = 0

        pwm31.start(pwm_val)
        pwm37.start(pwm_val)

        while counter_r < encoder_count:
            # ser.readline()
            #print(counter_r)
            # ticks_r.append(gpio.input(12))
            # ticks_l.append(gpio.input(7))

            if gpio.input(12) != tick_r:
                counter_r += 1
                tick_r = gpio.input(12)

            if gpio.input(7) != tick_l:
                counter_l += 1
                tick_l = gpio.input(7)
            
            error = counter_r - counter_l
            Kp = -2.5
            val = pwm_val + (Kp*error)
            if val > 100:
                val = 100
            if val < 0:
                val = 0
            pwm37.ChangeDutyCycle(val)

        pwm31.stop()
        pwm37.stop()

    def reverse(encoder_count):
        gpio.output(31,False)
        gpio.output(33,True)
        gpio.output(35,True)
        gpio.output(37,False)
        time.sleep(encoder_count)
        gameover()

    def right(angle_turn):
        # gpio.output(31,True)        
        # gpio.output(33,False)
        # gpio.output(35,True)
        # gpio.output(37,False)

        data = ser.readline()
        data = data.decode()
        try:
            initial_angle = float(data.split(" ")[1][:-4])
        except:
            initial_angle = 0
        current_angle = initial_angle

        if angle_turn < 180:
            check_angle = 180
        else:
            check_angle = 360

        pwm31.start(90)
        pwm35.start(90)

        while round(current_angle - initial_angle) % check_angle < angle_turn:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm31.stop()
        pwm35.stop()

        gameover()

    def left(angle_turn):
        #gpio.output(31,False)        
        #gpio.output(33,True)
        #gpio.output(35,False)
        #gpio.output(37,True)

        data = ser.readline()
        data = data.decode()
        try:
            initial_angle = float(data.split(" ")[1][:-4])
        except:
            initial_angle = 0
        current_angle = initial_angle

        if angle_turn < 180:
            check_angle = 180
        else:
            check_angle = 360
        
        pwm33.start(90)
        pwm37.start(90)

        while round(initial_angle - current_angle) % check_angle < angle_turn:
            data = ser.readline()
            data = data.decode()
            #print(data)
            current_angle = float(data.split(" ")[1][:-4])


        pwm33.stop()
        pwm37.stop()
        #gameover()

    def send_email(image):
        time_now = datetime.now().strftime('%Y%m%d%H%M%S')

        smtpUser = 'ENPM701.lorocks@gmail.com'
        smtpPass = 'jtesrcwsaygtxubj'


        buffer = cv2.imencode('.jpg', image)[1].tobytes()


        to = 'ENPM809TS19@gmail.com'
        fromAdd = smtpUser
        # cc = ['jsuriya@umd.edu']
        cc = ['lorocks@umd.edu']
        msg = MIMEMultipart()
        msg['Subject'] = f'ENPM701-HW9-BlockRetrieved-{time_now}-Lowell_Lobo-lorocks'
        msg['From'] = fromAdd
        msg['To'] = to
        msg['CC'] = cc
        msg.preamble = 'Image from RPi'

        body = MIMEText(f'Image Found at {time_now}')
        msg.attach(body)

        img = MIMEImage(buffer)
        msg.attach(img)

        s = smtplib.SMTP('smtp.gmail.com', 587)

        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(smtpUser, smtpPass)
        s.sendmail(fromAdd, [to] + cc, msg.as_string())
        s.quit()

        print("Email send")

    def findDistanceToBlock(height):
        if (height < 26):
            d = ((51-46) * (height - 24) / (24-26)) + 51
        elif (height < 27):
            d = ((3) * (height - 26) / (26-27)) + 46
        elif (height < 28):
            d = ((3) * (height - 27) / (27-28)) + 43
        elif (height < 33):
            d = ((3) * (height - 28) / (28-33)) + 40
        elif (height < 36):
            d = ((3) * (height - 33) / (33-36)) + 37
        elif (height < 39):
            d = ((3) * (height - 36) / (36-39)) + 34
        elif (height < 43):
            d = ((3) * (height - 39) / (39-43)) + 31
        elif (height < 51):
            d = ((3) * (height - 43) / (43-51)) + 28
        elif (height < 56):
            d = ((3) * (height - 51) / (51-56)) + 25
        elif (height < 65):
            d = ((3) * (height - 56) / (56-65)) + 22
        elif (height < 77):
            d = ((3) * (height - 65) / (65-77)) + 19
        elif (height < 90):
            d = ((3) * (height - 77) / (77-90)) + 16
        elif (height < 119):
            d = ((3) * (height - 90) / (90-119)) + 13
        elif (height < 185):
            d = ((3) * (height - 119) / (119-185)) + 10

        else:
            return -1

        return d
    
    def approachObject(y, height, width):
        if y + height > 350 and width > 200:
            return False
        return True
        
    object_in_servo = False
    approach = False

    while True:
        frame = videostream.read()

        image = cv2.flip(frame,-1)

        # Only mask if object not in servo, object_in_servo = False
        # When object_in_servo = True, navigate to drop zone
        if object_in_servo:
            break

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower[block_color], upper[block_color])

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)

            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 2)
            cv2.imshow("Test", image)

            if x > 320 - 5 or x + w < 320 + 5:
                ser.reset_input_buffer()
                x_centr = x + (w/2)

                x_diff = 320 - x_centr

                if x_diff < 0:
                    right(abs(x_diff*0.0061))
                else:
                    left(abs(x_diff*0.0061))
            else:
                print(y+h, w)
                dist_to_block = findDistanceToBlock(h)
                if dist_to_block == -1 or approach:
                    approach = approachObject(y, h, w)
                    print(approach)
                    if not approach:
                        print("end")
                        pwm_servo.ChangeDutyCycle(close - 0.5)
                        object_in_servo = True
                        
                        # Send EMail
                        time_now = datetime.now().strftime('%Y%m%d%H%M%S')

                        smtpUser = 'ENPM701.lorocks@gmail.com'
                        smtpPass = 'jtesrcwsaygtxubj'


                        buffer = cv2.imencode('.jpg', image)[1].tobytes()


                        to = 'ENPM809TS19@gmail.com'
                        fromAdd = smtpUser
                        # cc = ['jsuriya@umd.edu']
                        cc = ['lorocks@umd.edu']
                        msg = MIMEMultipart()
                        msg['Subject'] = f'ENPM701-HW9-BlockRetrieved-{time_now}-Lowell_Lobo-lorocks'
                        msg['From'] = fromAdd
                        msg['To'] = to
                        msg['CC'] = cc
                        msg.preamble = 'Image from RPi'

                        body = MIMEText(f'Image Found at {time_now}')
                        msg.attach(body)

                        img = MIMEImage(buffer)
                        msg.attach(img)

                        s = smtplib.SMTP('smtp.gmail.com', 587)

                        s.ehlo()
                        s.starttls()
                        s.ehlo()

                        s.login(smtpUser, smtpPass)
                        s.sendmail(fromAdd, [to] + cc, msg.as_string())
                        s.quit()

                        print("Email send")

                        reverse(2) # reverse then object in serv ocondition will take it to drop zone
                        break
                    else:
                        print("forward")
                        forward(20)
                else:
                    if not approach:
                        forward(int((motor_rots*encoder_tick*(dist_to_block - 5))/(2*3.1415*wheel_radius))) # save this distance travel so to use when reversing
                        pwm_servo.ChangeDutyCycle(open_s)
                        approach = True

            # Check for object at bottom screen              

            


        result.write(image)

        

        # Press key q to stop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    pwm_servo.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()
    result.release()
    cv2.destroyAllWindows()
    videostream.stop()
except Exception as e:
    print(e)
    pwm_servo.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()
    result.release()
    cv2.destroyAllWindows()
    videostream.stop()