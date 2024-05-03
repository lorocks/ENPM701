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
printed = False

count = 0

while not printed:
    if ser.in_waiting > 0:
        count += 1
        data = ser.readline()

        if count > 10:
            printed = True

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

gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)

print(f"Current Orientation: {data}")

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
            Kp = -1.6
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
        data = ser.readline()
        data = data.decode()
        try:
            angle = float(data.split(" ")[1][:-4])
        except:
            angle = -1

        # ser.reset_input_buffer()
        # time.sleep(0.1)
        # got_angle = False
        # angle = -1
        
        # while got_angle:
        #     data = ser.readline()
        #     data = data.decode()
        #     try:
        #         angle = float(data.split(" ")[1][:-4])
        #         got_angle = True
        #     except:
        #         pass

        return angle
    
    def getquad(angle):
        angle = round(angle)
        if angle <= 90:
            return 4
        elif angle <=180:
            return 3
        elif angle <= 270:
            return 2
        elif angle < 360:
            return 1

        return 4
    
    
    path_points = runPlanner(2)

    x_pos = path_points[0][0]
    y_pos = path_points[0][1]

    division = [2, 1]

    for location in path_points[1:]:
        for i in division:
            gottem = False
            dist = heuristic(location, (x_pos, y_pos))
            desired_angle = math.degrees(math.atan2(location[1] - y_pos, location[0] - x_pos)) % 360
            desired_angle = 360 - desired_angle
            while not gottem:
                angle = getangle()
                if angle != -1:
                    gottem = True

            print(f"Current: {angle}, Desired: {desired_angle}")

            angle_quad = getquad(angle)
            desired_quad = getquad(desired_angle)

            if desired_quad == 1 and angle_quad == 2:
                righttill(desired_angle)
            elif desired_quad == 1 and angle_quad == 4:
                lefttill(desired_angle)
            elif desired_quad == 2 and angle_quad == 3:
                righttill(desired_angle)
            elif desired_quad == 2 and angle_quad == 1:
                lefttill(desired_angle)
            elif desired_quad == 3 and angle_quad == 4:
                righttill(desired_angle)
            elif desired_quad == 3 and angle_quad == 2:
                lefttill(desired_angle)
            elif desired_quad == 4 and angle_quad == 1:
                righttill(desired_angle)
            elif desired_quad == 4 and angle_quad == 3:
                lefttill(desired_angle)
            elif desired_quad == angle_quad:
                if desired_angle - angle < 0:
                    righttill(desired_angle)
                else:
                    lefttill(desired_angle)
            else:
                if (180 - desired_angle) - angle < 0:
                    righttill(desired_angle)
                else:
                    lefttill(desired_angle)

            forward(int((motor_rots*encoder_tick*(dist/i))/(2*3.1415*wheel_radius)))

            x_pos += (dist/i) * math.cos((360 - angle) * math.pi / 180)
            y_pos += (dist/i) * math.sin((360 - angle) * math.pi / 180)


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