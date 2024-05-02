# RPi Stuff
import RPi.GPIO as gpio
import time

# Planner Functions
from planner_indoor_fn import runPlanner, heuristic
import math

# IMU Serial
import serial

# Initialise Serial
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Initialise Robot Params
wheel_radius = 1.3
encoder_tick = 8
motor_rots = 120

servo = 36

pwm_val = 75
Kp = -1.9

gpio.setmode(gpio.BOARD)
gpio.setup(servo,gpio.OUT)

gpio.setup(31,gpio.OUT) #IN1
gpio.setup(33,gpio.OUT) #IN2
gpio.setup(35,gpio.OUT) #IN3
gpio.setup(37,gpio.OUT)

pwm_servo = gpio.PWM(servo,50)
pwm_servo.start(7.9)

pwm31 = gpio.PWM(31, 50)
pwm33 = gpio.PWM(33, 50)
pwm35 = gpio.PWM(35, 50)
pwm37 = gpio.PWM(37, 50)


try:
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
    
    def getangle():
        ser.reset_input_buffer()
        time.sleep(0.1)
        got_angle = False
        
        while got_angle:
            data = ser.readline()
            data = data.decode()
            try:
                angle = float(data.split(" ")[1][:-4])
                return angle
            except:
                pass
    
    
    path_points = runPlanner(1/2)

    x_pos = path_points[0][0]
    y_pos = path_points[0][1]

    for location in path_points[1:]:
        for i in range(2):
            dist = heuristic(location, (x_pos, y_pos))
            desired_angle = math.degrees(math.atan2(location[1] - y_pos, location[0] - y_pos))

            # Need to think of logic for turning properly, based on lowesy angle cost to get there and not some nonse lik this
            angle_error = desired_angle - round(getangle())

            if angle_error < 0:
                angle = righttill(desired_angle)
            elif angle_error > 0:
                angle = lefttill(desired_angle)

            forward(int((motor_rots*encoder_tick*(dist/2))/(2*3.1415*wheel_radius)))

            x_pos += (dist/2) * math.cos((360 - angle) * math.pi / 180)
            y_pos += (dist/2) * math.sin((360 - angle) * math.pi / 180)


    pwm_servo.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()

except Exception as e:
    print(e)
    pwm_servo.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()