import RPi.GPIO as gpio
import time
import cv2
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

open = 5
close = 7.9

pwm_val = 50
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

try:
        
    def gameover():
        gpio.output(31,False)
        gpio.output(33,False)
        gpio.output(35,False)
        gpio.output(37,False)
        time.sleep(0.5)
        
    def forward(encoder_count):
        counter_r = counter_l = 0
        tick_r = tick_l = 0
        ticks_r = ticks_l = []

        pwm31.start(pwm_val)
        pwm37.start(pwm_val)

        while counter_r < encoder_count:
            ticks_r.append(gpio.input(12))
            ticks_l.append(gpio.input(7))

            if gpio.input(12) != tick_r:
                counter_r += 1
                tick_r = gpio.input(12)

            if gpio.input(7) != tick_l:
                counter_l += 1
                tick_l = gpio.input(7)
            
            error = counter_r - counter_l

            val = pwm_val + (Kp*error)
            if val > 100:
                val = 100
            if val < 0:
                val = 0
            pwm37.ChangeDutyCycle(val)

        gameover()

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

        while (initial_angle - current_angle) % 360 >= angle_turn:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])

        gameover()

    def left(angle_turn):
        gpio.output(31,False)
        gpio.output(33,True)
        gpio.output(35,False)
        gpio.output(37,True)

        data = ser.readline()
        data = data.decode()
        initial_angle = float(data.split(" ")[1][:-4])
        current_angle = initial_angle

        while (initial_angle - current_angle) % 360 >= angle_turn:
            data = ser.readline()
            data = data.decode()
            current_angle = float(data.split(" ")[1][:-4])

        gameover()
        
    counter = 0

    ser.readline()

    # Time sleep gonna mess up readline
    forward(120*2*2)
    time.sleep(2)
    left(90)
    time.sleep(2)
    forward(120*2*2)
    time.sleep(2)
    left(90)
    time.sleep(2)
    forward(120*2*2)
    time.sleep(2)
    left(90)
    time.sleep(2)
    forward(120*2*2)
    time.sleep(2)
    left(90)
    time.sleep(2)




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
    gpio.cleanup()