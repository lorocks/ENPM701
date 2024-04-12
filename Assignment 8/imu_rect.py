import RPi.GPIO as gpio
import time
import cv2
import os
import serial

from threading import Thread


ser = serial.Serial('/dev/ttyUSB0', 9600)

printed = False

count = 0

while not printed:
    if ser.in_waiting > 0:
        count += 1
        ser.readline()

        if count > 10:
            printed = True


trig = 16
echo = 18
servo = 36

open_s = 5
close = 7.9

pwm_val = 75
Kp = -2.1

gpio.setmode(gpio.BOARD)
gpio.setup(servo,gpio.OUT)

gpio.setup(31,gpio.OUT) #IN1
gpio.setup(33,gpio.OUT) #IN2
gpio.setup(35,gpio.OUT) #IN3
gpio.setup(37,gpio.OUT)

pwm = gpio.PWM(servo,50)
pwm.start(close)

pwm31 = gpio.PWM(31, 50)
pwm33 = gpio.PWM(33, 50)
pwm35 = gpio.PWM(35, 50)
pwm37 = gpio.PWM(37, 50)

gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)

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
        time.sleep(1)
        gameover()

    def right(angle_turn):
        gpio.output(31,True)        
        gpio.output(33,False)
        gpio.output(35,True)
        gpio.output(37,False)

        data = ser.readline()
        data = data.decode()
        initial_angle = float(data.split(" ")[1][:-4])
        current_angle = initial_angle

        if angle_turn < 180:
            check_angle = 180
        else:
            check_angle = 360

        while (initial_angle - current_angle) % check_angle >= angle_turn:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])

        gameover()

    def left(angle_turn):
        #gpio.output(31,False)        
        #gpio.output(33,True)
        #gpio.output(35,False)
        #gpio.output(37,True)
        
        pwm33.start(90)
        pwm37.start(90)

        data = ser.readline()
        data = data.decode()
        initial_angle = float(data.split(" ")[1][:-4])
        current_angle = initial_angle

        if angle_turn < 180:
            check_angle = 180
        else:
            check_angle = 360

        while round(initial_angle - current_angle) % check_angle < angle_turn - 1:
            data = ser.readline()
            data = data.decode()
            #print(data)
            current_angle = float(data.split(" ")[1][:-4])

            if (initial_angle - current_angle) % check_angle > angle_turn - 30:
                pwm33.ChangeDutyCycle(60)
                pwm37.ChangeDutyCycle(60)

        print(current_angle)
        pwm33.stop()
        pwm37.stop()
        #gameover()
        
    counter = 0

    ser.readline()

    angles = []

    data = ser.readline()
    data = data.decode()
    angles.append(float(data.split(" ")[1][:-4]))
    
    forward(120*8*2)
    timeskip(1.5)
    left(90)
    timeskip(1.5)

    data = ser.readline()
    data = data.decode()
    angles.append(float(data.split(" ")[1][:-4]))

    forward(120*8*2)
    timeskip(1.5)
    left(90)
    timeskip(1.5)

    data = ser.readline()
    data = data.decode()
    angles.append(float(data.split(" ")[1][:-4]))

    forward(120*8*2)
    timeskip(1.5)
    left(89)
    timeskip(1.5)

    data = ser.readline()
    data = data.decode()
    angles.append(float(data.split(" ")[1][:-4]))

    forward(120*8*2)
    timeskip(1.5)
    timeskip(1.5)


    if os.path.exists("imu_angles.txt"):
        os.remove("imu_angles.txt")
    else:
        print("The file does not exist")

    f = open("imu_angles.txt", 'w')
    for i in angles:
        f.write(str(i))
        f.write('\n')
    f.close()


    pwm.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gpio.cleanup()
except Exception as error:
    print("Error aborting!", error)
    pwm.stop()
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()
    gameover()
    gpio.cleanup()
