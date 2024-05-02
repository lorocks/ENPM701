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
lower_green = np.array([37, 85, 100])
upper_green = np.array([60, 255, 255])

lower_blue = np.array([105, 90, 30])
upper_blue = np.array([130, 255, 255])

lower_red = np.array([150, 100, 100])
upper_red = np.array([200, 255, 255])

# Change order based on actual application
lower = [lower_green, lower_red]
upper = [upper_green, upper_red]

# For block rotation
current_block = 0

# FSM
state = 0

# Location in Grid
x_pos = 0
y_pos = 0
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
        ser.reset_input_buffer()
        time.sleep(0.1)
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
        ser.reset_input_buffer()
        time.sleep(0.1)
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
            Kp = -1.9
            val = pwm_val + (Kp*error)
            if val > 100:
                val = 100
            if val < 0:
                val = 0
            pwm37.ChangeDutyCycle(val)

        pwm31.stop()
        pwm37.stop()
    
    def lefttill(angle):
        ser.reset_input_buffer()
        time.sleep(0.1)
        data = ser.readline()
        data = data.decode()
        try:
            initial_angle = float(data.split(" ")[1][:-4])
        except:
            initial_angle = 0
        current_angle = initial_angle
        
        pwm33.start(90)
        pwm37.start(90)

        while current_angle < angle - 2 or current_angle > angle + 2:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm33.stop()
        pwm37.stop()

        return current_angle
    
    def righttill(angle):
        ser.reset_input_buffer()
        time.sleep(0.1)
        data = ser.readline()
        data = data.decode()
        try:
            initial_angle = float(data.split(" ")[1][:-4])
        except:
            initial_angle = 0
        current_angle = initial_angle
        
        pwm31.start(95)
        pwm35.start(95)

        while current_angle < angle - 2 or current_angle > angle + 2:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])


        pwm31.stop()
        pwm35.stop()

        return current_angle

    
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

    first_dist = 10
    first_find = 0

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

            
            if len(contours) > 0:
                # assuming blue is third
                if current_block % 3 == 3:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                x_centr = x + (w/2)

                x_diff = 320 - x_centr

                if x_diff < 0:
                    angle = right(abs(x_diff*0.051))
                else:
                    angle = left(abs(x_diff*0.051))
                forward(int((motor_rots*encoder_tick*(first_dist))/(2*3.1415*wheel_radius))) ##### For now move 1 foot

                x_pos += first_dist * math.cos((360 - angle) * math.pi / 180)
                y_pos += first_dist * math.sin((360 - angle) * math.pi / 180)

                print(state)
                state += 1

            else:
                first_dist = 0

                if current_block == 0 and first_find == 0:
                    if current_block == 0:
                        angle = lefttill(51)
                    else:
                        angle = righttill(360 - 51)
                    forward(int((motor_rots*encoder_tick*(70))/(2*3.1415*wheel_radius)))
                    x_pos += 70 * math.cos((360 - angle) * math.pi / 180)
                    y_pos += 70 * math.sin((360 - angle) * math.pi / 180)
                    angle = righttill(330)
                elif first_find == 0:
                    if current_block == 0:
                        angle = lefttill(51)
                    else:
                        angle = righttill(360 - 51)
                    forward(int((motor_rots*encoder_tick*(70))/(2*3.1415*wheel_radius)))
                    x_pos += 70 * math.cos((360 - angle) * math.pi / 180)
                    y_pos += 70 * math.sin((360 - angle) * math.pi / 180)
                    angle = lefttill(330) # go to 330
                elif first_find == 1:
                    angle = lefttill(30)
                elif first_find == 2:
                    angle = righttill(2)
                elif first_find == 3:
                    forward(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                    x_pos += 6 * math.cos((360 - angle) * math.pi / 180)
                    y_pos += 6 * math.sin((360 - angle) * math.pi / 180)
                    righttill(360 - 45)
                    first_find = 0

                first_find += 1         
        
        # Move closer to block based on estimate location
        elif state == 1:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 3:
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
                        forward(int((motor_rots*encoder_tick*(d/3 - 12))/(2*3.1415*wheel_radius)))
                        x_pos += (d/3 - 12) * math.cos((360 - angle) * math.pi / 180)
                        y_pos += (d/3 - 12) * math.sin((360 - angle) * math.pi / 180)

                        print(state)
                        state += 1
                else:
                    if first_dist == 0:
                        forward(int((motor_rots*encoder_tick*(3))/(2*3.1415*wheel_radius)))
                        x_pos += 3 * math.cos((360 - angle) * math.pi / 180)
                        y_pos += 3 * math.sin((360 - angle) * math.pi / 180)
                    else:
                        forward(int((motor_rots*encoder_tick*(6))/(2*3.1415*wheel_radius)))
                        x_pos += 6 * math.cos((360 - angle) * math.pi / 180)
                        y_pos += 6 * math.sin((360 - angle) * math.pi / 180)
            

        # Perfectly Orient and move closest
        elif state == 2:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 3:
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
                    ##### maybe just have to minus distance from cam to block
                    d = findDistanceToBlock(h)
                    forward(int((motor_rots*encoder_tick*(d - 11))/(2*3.1415*wheel_radius)))
                    pwm_servo.ChangeDutyCycle(open_s)
                    x_pos += (d - 10) * math.cos((360 - angle) * math.pi / 180)
                    y_pos += (d - 10) * math.sin((360 - angle) * math.pi / 180)

                    print(state)
                    state += 1
            

        # Approach and grab
        elif state == 3:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower[current_block % 3], upper[current_block % 3])

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                if current_block % 3 == 3:
                    cnt = contours[0]
                    x,y,w,h = cv2.boundingRect(cnt)
                else:
                    c = max(contours, key = cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)

                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 2)

                if x + (w*3/10) > 320  or x + w - (w*3/10) < 320:
                    x_centr = x + (w/2)

                    x_diff = 320 - x_centr

                    if x_diff < 0:
                        angle = right(abs(x_diff*0.0061))
                    else:
                        angle = left(abs(x_diff*0.0061))
                else:
                    d_ = findDistanceToBlock(h) ##### Do something maybe idk
                    approach = approachObject(y, h, w)
                    if not approach:
                        pwm_servo.ChangeDutyCycle(close - 0.5)
                        time.sleep(0.25)

                        print(state)
                        state += 1
                    else:
                        forward(20)
                        d_ = 20 * (2*3.1415*wheel_radius) / (motor_rots*encoder_tick)
                        x_pos += d_ * math.cos((360 - angle) * math.pi / 180)
                        y_pos += d_ * math.sin((360 - angle) * math.pi / 180)
                    
            
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

                print(state)
                state += 1
            

        # Reverse out of clump
        elif state == 5:
            # use x and y pos here to ensure action not too far going
            x_test = x_pos + (d * math.cos((360 - angle + 180) * math.pi / 180))
            y_test = y_pos + (d * math.sin((360 - angle + 180) * math.pi / 180))
            if x_test > 0 and y_test > 0 and x_test < 120 and y_test < 120:
                reverse(int((motor_rots*encoder_tick*(d))/(2*3.1415*wheel_radius)))
                x_pos = x_test
                y_pos = y_test
            else:
                print(state)
                state += 1

        # Turn and approach x value
        elif state == 6:
            ser.reset_input_buffer()
            time.sleep(0.1)
            data = ser.readline()
            data = data.decode()
            angle_b = float(data.split(" ")[1][:-4])

            angle = lefttill(180)

            angle_diff = 180 - angle
            if angle_diff < 0:
                right(angle_diff)
            else:
                left(angle_diff)

            movetill(int((motor_rots*encoder_tick*(x_pos))/(2*3.1415*wheel_radius)), 49)
            x_pos = 49 * 0.393701 # this is wrong, need to get x_pos using encoder data, return imu also

            if x_pos < 24:
                print(state)
                state += 1

        # Turn and approach y value
        elif state == 7:
            angle = righttill(85)
            movetill(int((motor_rots*encoder_tick*(120))/(2*3.1415*wheel_radius)), 40)
            y_pos = 120 - (40 * 0.393701) # this is wrong, need to get y_pos using encoder data, return imu also

            if y_pos > 120 - 24:
                print(state)
                state += 1

        # Place object
        elif state == 8:
            # before drop put a state to check actuality
            pwm_servo.ChangeDutyCycle(open_s)
            current_block += 1

            # End condition
            if current_block >= 9:
                state = 10
            else:
                print(state)
                state += 1

        # Wiggle Wiggle for Start
        elif state == 9:
            reverse(int((motor_rots*encoder_tick*(120 - y_pos))/(2*3.1415*wheel_radius))) # small amount
            y_pos -= 120 - y_pos
            pwm_servo.ChangeDutyCycle(close)
            angle = lefttill(350) # choose between left right which faster
            forward(int((motor_rots*encoder_tick*(12))/(2*3.1415*wheel_radius)))
            x_pos += (12) * math.cos((360 - angle) * math.pi / 180)
            y_pos += (12) * math.sin((360 - angle) * math.pi / 180)
            
            print(state)
            state = 0

            first_dist = 10
            first_find = 0

        # Task completed
        elif state == 10:
            print("Task Completed")

            pwm_servo.stop()
            pwm31.stop()
            pwm33.stop()
            pwm35.stop()
            pwm37.stop()
            gpio.cleanup()
            result.release()
            cv2.destroyAllWindows()
            videostream.stop()

            break  

        cv2.imshow("Frame", image)          
        
        ##### For move_till might need a 10 values check for ultrasonic
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