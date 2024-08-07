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
printed = False

### Testing this removal
# count = 0

# while not printed:
#     if ser.in_waiting > 0:
#         count += 1
#         print(ser.readline())

#         if count > 10:
#             printed = True



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


# Initialise HSV Colors
lower_green = np.array([37, 65, 100])
upper_green = np.array([60, 255, 255])

lower_blue = np.array([105, 90, 30])
upper_blue = np.array([130, 255, 255])

lower_red = np.array([150, 100, 100])
upper_red = np.array([200, 255, 255])

# Change order based on actual application
lower = [lower_red, lower_green, lower_blue]
upper = [upper_red, upper_green, upper_blue]

# For block rotation
current_block = 0

# FSM
state = 0

# Location in Grid
x_pos = 0
y_pos = 15
angle = 0

# Ultrsonic distances
u_x_dist = [40, 43, 46, 49, 40, 43, 46, 49, 45]
u_y_dist = [43, 43, 43, 43, 46, 46, 46, 46, 45]

try:
    def wall_dist():
        gpio.output(trig, True)
        time.sleep(0.00001)
        gpio.output(trig, False)

        while gpio.input(echo) == 0:
            start = time.time()

        while gpio.input(echo) == 1:
            end = time.time()

        try:
            duration = end - start

            distance = duration * 17150

            return distance
        except:
            return 400

    def gameover():
        gpio.output(31,False)
        gpio.output(33,False)
        gpio.output(35,False)
        gpio.output(37,False)

    def forward(encoder_count):
        counter_r = counter_l = 0
        tick_r = tick_l = 0

        pwm31.start(pwm_val)
        pwm37.start(pwm_val)

        while counter_r < encoder_count:
            if gpio.input(12) != tick_r:
                counter_r += 1
                tick_r = gpio.input(12)

            if gpio.input(7) != tick_l:
                counter_l += 1
                tick_l = gpio.input(7)
            
            error = counter_r - counter_l
            Kp = -1.9
            val = pwm_val + (Kp*error)
            if val > 100:
                val = 100
            if val < 0:
                val = 0
            pwm37.ChangeDutyCycle(val)

        pwm31.stop()
        pwm37.stop()

    def reverse(encoder_count):
        counter_r = counter_l = 0
        tick_r = tick_l = 0

        pwm33.start(pwm_val)
        pwm35.start(pwm_val)

        while counter_r < encoder_count or counter_l < encoder_count:
            if gpio.input(12) != tick_r:
                counter_r += 1
                tick_r = gpio.input(12)

            if gpio.input(7) != tick_l:
                counter_l += 1
                tick_l = gpio.input(7)
            

        pwm33.stop()
        pwm35.stop()

    def right(angle_turn):
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

        if angle_turn < 180:
            check_angle = 180
        else:
            check_angle = 360

        pwm31.start(95)
        pwm35.start(95)

        while round(current_angle - initial_angle) % check_angle < angle_turn:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm31.stop()
        pwm35.stop()

        return current_angle

    def left(angle_turn):
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

        if angle_turn < 180:
            check_angle = 180
        else:
            check_angle = 360
        
        pwm33.start(90)
        pwm37.start(90)

        while round(initial_angle - current_angle) % check_angle < angle_turn:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm33.stop()
        pwm37.stop()

        return current_angle

    def send_email(image, x, y):
        time_now = datetime.now().strftime('%Y%m%d%H%M%S')

        smtpUser = 'ENPM701.lorocks@gmail.com'
        smtpPass = 'jtesrcwsaygtxubj'

        buffer = cv2.imencode('.jpg', image)[1].tobytes()

        to = 'ENPM809TS19@gmail.com'
        fromAdd = smtpUser
        cc = 'jsuriya@umd.edu,lorocks@umd.edu'
        msg = MIMEMultipart()
        msg['Subject'] = f'ENPM701-HW9-BlockRetrieved-{time_now}-Lowell_Lobo-lorocks'
        msg['From'] = fromAdd
        msg['To'] = to
        msg['Cc'] = cc
        msg.preamble = 'Image from RPi'

        body = MIMEText(f'Image Found at {time_now}')
        msg.attach(body)

        img = MIMEImage(buffer)
        msg.attach(img)

        body = MIMEText(f'Block was at position ({round(x, 2)}, {round(y, 2)})')
        msg.attach(body)

        s = smtplib.SMTP('smtp.gmail.com', 587)

        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(smtpUser, smtpPass)
        # s.sendmail(fromAdd, to.split(",") + cc.split(","), msg.as_string())
        s.sendmail(fromAdd, to.split(","), msg.as_string())
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
    
    # Check if should pick object
    def approachObject(y, height, width):
        if y + height > 350 and width > 200:
            return False
        return True
    
    # Forward until reaching wall
    def movetill(encoder_count, u_dist):
        counter_r = counter_l = 0
        tick_r = tick_l = 0

        dist = wall_dist()

        pwm31.start(pwm_val)
        pwm37.start(pwm_val)

        while counter_r < encoder_count and dist > u_dist:
            d = []
            if gpio.input(12) != tick_r:
                counter_r += 1
                tick_r = gpio.input(12)

            if gpio.input(7) != tick_l:
                counter_l += 1
                tick_l = gpio.input(7)
            
            for i in range(10):
                d.append(wall_dist())
            d = sorted(d)
            dist = (d[4] + d[5])/2
            
            error = counter_r - counter_l
            Kp = -1.6
            val = pwm_val + (Kp*error)
            if val > 100:
                val = 100
            if val < 0:
                val = 0
            pwm37.ChangeDutyCycle(val)

        pwm31.stop()
        pwm37.stop()

        return (counter_l + counter_r)/2, dist
    
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
    
    def righttill(angle):
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
        
        pwm31.start(95)
        pwm35.start(95)

        while current_angle < angle - 4 or current_angle > angle + 4:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm31.stop()
        pwm35.stop()

        return current_angle
    
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

    
    # Initialize the webcam
    videostream = VideoStream().start()
    time.sleep(1)

    # Initialise Video Writer
    frame_width = int(640)
    frame_height = int(480)

    size = (frame_width, frame_height)

    # result = cv2.VideoWriter('my_video.avi',
    #                         cv2.VideoWriter_fourcc(*'MJPG'),
    #                         5, size)

    first_dist = 10
    first_find = 0
    try_count = 0


    for i in range(10):
        print(getangle())

    begin = input("Enter to start: ")

    print("Starting")
    while True:
        frame = videostream.read()

        image = cv2.flip(frame, -1)

        # Press key q to stop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Move toward block on image quadrant
        if state == 0:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            if current_block % 3 == 2 and first_dist > 0: ## Later
                angle = righttill(51)
                forward(int((motor_rots*encoder_tick*(60))/(2*3.1415*wheel_radius)))
                x_pos += 60 * math.cos((360 - angle) * math.pi / 180)
                y_pos += 60 * math.sin((360 - angle) * math.pi / 180)

                first_find += 1
                first_dist = 0

            elif len(contours) > 0:
                # assuming blue is third
                if current_block % 3 == 2:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                if current_block % 3 == 2 and y+h/480 < 0.35:
                    if first_find == 1:
                        angle = lefttill(360 - 45)
                    elif first_find == 2:
                        angle = righttill(2)
                    elif first_find == 3:
                        gottem = False
                        while not gottem:
                            angle = getangle()
                            if angle != -1:
                                gottem = True  
                        forward(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                        x_pos += 6 * math.cos((360 - angle) * math.pi / 180)
                        y_pos += 6 * math.sin((360 - angle) * math.pi / 180)
                        righttill(45)
                        first_find = 0

                    first_find += 1 

                else:
                    x_centr = x + (w/2)

                    x_diff = 320 - x_centr

                    print(f"Difference {x_diff}")
                    
                    if round(abs(x_diff)) != 0 and abs(x_diff) > 160:
                        if x_diff < 0:
                            angle = right(abs(x_diff*0.071/2))
                        else:
                            angle = left(abs(x_diff*0.071/2))
                    gottem = False
                    while not gottem:
                        angle = getangle()
                        if angle != -1:
                            gottem = True  
                    forward(int((motor_rots*encoder_tick*(first_dist))/(2*3.1415*wheel_radius)))

                    x_pos += first_dist * math.cos((360 - angle) * math.pi / 180)
                    y_pos += first_dist * math.sin((360 - angle) * math.pi / 180)

                    print(angle)

                    
                    state += 1
                    print(state)

            else:
                first_dist = 0

                if current_block == 0 and first_find == 0:
                    if current_block == 0:
                        print("here")
                        angle = lefttill(360 - 51)
                        print("huh")
                    else:
                        angle = righttill(51)
                    gottem = False
                    while not gottem:
                        angle = getangle()
                        if angle != -1:
                            gottem = True  
                    forward(int((motor_rots*encoder_tick*(60))/(2*3.1415*wheel_radius)))
                    x_pos += 60 * math.cos((360 - angle) * math.pi / 180)
                    y_pos += 60 * math.sin((360 - angle) * math.pi / 180)
                    angle = righttill(45)
                elif first_find == 0:
                    if current_block == 0:
                        angle = lefttill(360 - 51)
                    else:
                        angle = righttill(51)
                    gottem = False
                    while not gottem:
                        angle = getangle()
                        if angle != -1:
                            gottem = True  
                    forward(int((motor_rots*encoder_tick*(50))/(2*3.1415*wheel_radius)))
                    x_pos += 50 * math.cos((360 - angle) * math.pi / 180)
                    y_pos += 50 * math.sin((360 - angle) * math.pi / 180)
                elif first_find == 1:
                    angle = lefttill(360 - 45)
                elif first_find == 2:
                    angle = righttill(2)
                elif first_find == 3:
                    gottem = False
                    while not gottem:
                        angle = getangle()
                        if angle != -1:
                            gottem = True  
                    forward(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                    x_pos += 6 * math.cos((360 - angle) * math.pi / 180)
                    y_pos += 6 * math.sin((360 - angle) * math.pi / 180)
                    righttill(45)
                    first_find = 0

                first_find += 1     
        
        # Move closer to block based on estimate location
        elif state == 1:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 2:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                if h >= 40:
                    if x > 320  or x + w  < 320:
                        x_centr = x + (w/2)

                        x_diff = 320 - x_centr

                        if x_diff < 0:
                            angle = right(abs(x_diff*0.0061))
                        else:
                            angle = left(abs(x_diff*0.0061))
                    else:
                        d = findDistanceToBlock(h)
                        if not d/3 - 12 < 0:
                            gottem = False
                            while not gottem:
                                angle = getangle()
                                if angle != -1:
                                    gottem = True  
                            forward(int((motor_rots*encoder_tick*(d/3 - 12))/(2*3.1415*wheel_radius)))
                            x_pos += (d/3 - 12) * math.cos((360 - angle) * math.pi / 180)
                            y_pos += (d/3 - 12) * math.sin((360 - angle) * math.pi / 180)

                        state += 1
                        print(state)
                else:
                    if first_dist == 0:
                        gottem = False
                        while not gottem:
                            angle = getangle()
                            if angle != -1:
                                gottem = True  
                        forward(int((motor_rots*encoder_tick*(3))/(2*3.1415*wheel_radius)))
                        x_pos += 3 * math.cos((360 - angle) * math.pi / 180)
                        y_pos += 3 * math.sin((360 - angle) * math.pi / 180)
                    else:
                        gottem = False
                        while not gottem:
                            angle = getangle()
                            if angle != -1:
                                gottem = True  
                        forward(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                        x_pos += 6 * math.cos((360 - angle) * math.pi / 180)
                        y_pos += 6 * math.sin((360 - angle) * math.pi / 180)
            else:
                gottem = False
                while not gottem:
                    angle = getangle()
                    if angle != -1:
                        gottem = True  
                reverse(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                x_pos += (6) * math.cos((360 - angle + 180) * math.pi / 180)
                y_pos += (6) * math.sin((360 - angle + 180) * math.pi / 180)
                try_count += 1
                
            

        # Perfectly Orient and move closest
        elif state == 2:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 2:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 2)

                if x + (w/5) > 320  or x + w - (w/5) < 320:
                    x_centr = x + (w/2)

                    x_diff = 320 - x_centr

                    if x_diff < 0:
                        angle = right(abs(x_diff*0.0061))
                    else:
                        angle = left(abs(x_diff*0.0061))
                else:
                    ##### maybe just have to minus distance from cam to block, might need reorient, run state 2 twice ?
                    d = findDistanceToBlock(h)
                    gottem = False
                    while not gottem:
                        angle = getangle()
                        if angle != -1:
                            gottem = True  
                    forward(int((motor_rots*encoder_tick*(d/2 - 6))/(2*3.1415*wheel_radius))) ##### Need to test this
                    pwm_servo.ChangeDutyCycle(open_s)
                    x_pos += (d/2 - 6) * math.cos((360 - angle) * math.pi / 180)
                    y_pos += (d/2 - 6) * math.sin((360 - angle) * math.pi / 180)

                    state = 20
                    print(state)
            else:
                gottem = False
                while not gottem:
                    angle = getangle()
                    if angle != -1:
                        gottem = True  
                reverse(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                x_pos += (6) * math.cos((360 - angle + 180) * math.pi / 180)
                y_pos += (6) * math.sin((360 - angle + 180) * math.pi / 180)

        # Extra Perfectly Orient and move closest
        elif state == 20:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 2:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 2)

                if x + (w/5) > 320  or x + w - (w/5) < 320:
                    x_centr = x + (w/2)

                    x_diff = 320 - x_centr

                    if x_diff < 0:
                        angle = right(abs(x_diff*0.0061))
                    else:
                        angle = left(abs(x_diff*0.0061))
                else:
                    ##### maybe just have to minus distance from cam to block, might need reorient, run state 2 twice ?
                    d = findDistanceToBlock(h)
                    gottem = False
                    while not gottem:
                        angle = getangle()
                        if angle != -1:
                            gottem = True  
                    forward(int((motor_rots*encoder_tick*(d - 7))/(2*3.1415*wheel_radius))) ##### Need to test this
                    x_pos += (d - 7) * math.cos((360 - angle) * math.pi / 180)
                    y_pos += (d - 7) * math.sin((360 - angle) * math.pi / 180)

                    state = 3
                    print(state)
            else:
                gottem = False
                while not gottem:
                    angle = getangle()
                    if angle != -1:
                        gottem = True  
                reverse(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                x_pos += (6) * math.cos((360 - angle + 180) * math.pi / 180)
                y_pos += (6) * math.sin((360 - angle + 180) * math.pi / 180)
            

        # Approach and grab
        elif state == 3:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 2:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 2)

                if x < 160 or x > 320 +160:
                    reverse(int((motor_rots*encoder_tick*(d_))/(2*3.1415*wheel_radius)))
                    x_pos += (d_) * math.cos((360 - angle + 180) * math.pi / 180)
                    y_pos += (d_) * math.sin((360 - angle + 180) * math.pi / 180)
                

                if x + (w*3/10) > 320  or x + w - (w*3/10) < 320:
                    x_centr = x + (w/2)

                    x_diff = 320 - x_centr

                    if x_diff < 0:
                        angle = right(abs(x_diff*0.0061))
                    else:
                        angle = left(abs(x_diff*0.0061))
                else:
                    d_ = findDistanceToBlock(h)
                    approach = approachObject(y, h, w)
                    if not approach:
                        pwm_servo.ChangeDutyCycle(close - 0.5)
                        time.sleep(0.25)

                        state += 1
                        print(state)
                    else:
                        d_ = 45 * (2*3.1415*wheel_radius) / (motor_rots*encoder_tick)
                        gottem = False
                        while not gottem:
                            angle = getangle()
                            if angle != -1:
                                gottem = True  
                        forward(int((motor_rots*encoder_tick*(d_))/(2*3.1415*wheel_radius)))
                        # d_ = 45 * (2*3.1415*wheel_radius) / (motor_rots*encoder_tick)
                        x_pos += d_ * math.cos((360 - angle) * math.pi / 180)
                        y_pos += d_ * math.sin((360 - angle) * math.pi / 180)
            else:
                d_ = 45 * (2*3.1415*wheel_radius) / (motor_rots*encoder_tick)
                gottem = False
                while not gottem:
                    angle = getangle()
                    if angle != -1:
                        gottem = True  
                reverse(int((motor_rots*encoder_tick*(d_))/(2*3.1415*wheel_radius)))
                x_pos += (d_) * math.cos((360 - angle + 180) * math.pi / 180)
                y_pos += (d_) * math.sin((360 - angle + 180) * math.pi / 180)
                try_count += 1
                
            
        # Send email
        elif state == 4:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                c = max(contours, key = cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)

                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 2)

                send_email(image, x_pos, y_pos)

            state += 1
            print(state)
            

        # Reverse out of clump
        elif state == 5:
            # use x and y pos here to ensure action not too far going
            gottem = False
            while not gottem:
                angle = getangle()
                if angle != -1:
                    gottem = True  
            x_test = x_pos + (2*d * math.cos((360 - angle + 180) * math.pi / 180))
            y_test = y_pos + (2*d * math.sin((360 - angle + 180) * math.pi / 180))
            if x_test > 0 and y_test > 0 and x_test < 120 and y_test < 120:
                reverse(int((motor_rots*encoder_tick*(2*d))/(2*3.1415*wheel_radius)))
                x_pos = x_test
                y_pos = y_test

                state += 1
            else:
                x_test = x_pos + (d * math.cos((360 - angle + 180) * math.pi / 180))
                y_test = y_pos + (d * math.sin((360 - angle + 180) * math.pi / 180))
                if x_test > 0 and y_test > 0 and x_test < 120 and y_test < 120:
                    reverse(int((motor_rots*encoder_tick*(d))/(2*3.1415*wheel_radius)))
                    x_pos = x_test
                    y_pos = y_test
                state += 1
            print(state)


        ### Here is the optimization step
        # Turn and approach x value
        elif state == 6:
            # ser.reset_input_buffer()
            # time.sleep(0.1)
            # data = ser.readline()
            # data = data.decode()
            # angle_b = float(data.split(" ")[1][:-4])

            angle_ = math.degrees(math.atan2(108 - y_pos, 0 - x_pos)) % 360
            angle_ = 360 - angle_

            if angle_ > 188 and angle_ < 245:
                angle_ -= 1
            else:
                angle_ = 188

            angle = lefttill(angle_)

            # angle_diff = 180 - angle
            # if angle_diff < 0:
            #     angle = right(angle_diff)
            # else:
            #     angle = left(angle_diff)

            encoder_count, u_dist = movetill(int((motor_rots*encoder_tick*(x_pos))/(2*3.1415*wheel_radius)), 40)
            d_ = encoder_count * (2*3.1415*wheel_radius) / (motor_rots*encoder_tick)
            x_pos += d_ * math.cos((360 - angle) * math.pi / 180)

            gottem = False
            while not gottem:
                angle = getangle()
                if angle != -1:
                    gottem = True       

            ### Query angle again for proper location
            x_pos = math.cos(math.radians(abs(270 - angle))) * u_dist * 0.393701
            # x_pos = 49 * 0.393701
            state += 1
            print(state)

        # Turn and approach y value
        elif state == 7:
            angle = righttill(360 - 87.5)
            encoder_count, u_dist = movetill(int((motor_rots*encoder_tick*(120))/(2*3.1415*wheel_radius)), 45)
            d_ = encoder_count * (2*3.1415*wheel_radius) / (motor_rots*encoder_tick)
            y_pos += d_ * math.sin((360 - angle) * math.pi / 180)

            gottem = False
            while not gottem:
                angle = getangle()
                if angle != -1:
                    gottem = True      

            ### Query angle again for proper location
            y_pos = math.cos(math.radians(abs(angle - 180))) * u_dist * 0.393701
            state += 1
            print(state)

        # Place object
        elif state == 8:
            # before drop put a state to check actuality
            pwm_servo.ChangeDutyCycle(open_s)
            time.sleep(0.1)
            current_block += 1

            # End condition
            if current_block >= 9:
                state = 10
            else:
                state += 1
                print(state)

        # Wiggle Wiggle for Start
        elif state == 9:
            reverse(int((motor_rots*encoder_tick*(120 - y_pos))/(2*3.1415*wheel_radius)))
            y_pos -= 120 - y_pos
            pwm_servo.ChangeDutyCycle(close)
            angle = righttill(360 - 350) # choose between left right which faster
            forward(int((motor_rots*encoder_tick*(12))/(2*3.1415*wheel_radius)))
            x_pos += (12) * math.cos((360 - angle) * math.pi / 180)
            y_pos += (12) * math.sin((360 - angle) * math.pi / 180)
            
            state = 0
            print(state)

            first_dist = 10
            first_find = 0
            try_count = 0

        # Task completed
        elif state == 10:
            print("Task Completed")
            for i in range(10):
                print(getangle())

            break  

        cv2.imshow("Frame", image)          
        
    pwm_servo.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()
    # result.release()
    cv2.destroyAllWindows()
    videostream.stop()

except:
    pwm_servo.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()
    # result.release()
    cv2.destroyAllWindows()
    videostream.stop()
