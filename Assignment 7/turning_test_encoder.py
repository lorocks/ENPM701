import RPi.GPIO as gpio
import time
import os
import serial

# Better to use imu to get it :O
# 5 encoder ticks per angle
ser = serial.Serial('/dev/ttyUSB0', 9600)

try:
    # count = 0
    # printed = False
    # while not printed:
    #     if ser.in_waiting > 0:
    #         count += 1
    #         ser.readline()

    #         if count > 10:
    #             data = ser.readline()
    #             print(data)

    #             # data = float(str(data.rstrip().lstrip()).strip("'").strip("b'")[2:7])
    #             # print(data)

    #             printed = True

    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33, gpio.OUT) 
    gpio.setup(35, gpio.OUT)
    gpio.setup(37,gpio.OUT) #IN4
    gpio.setup(36, gpio.OUT)

    pwm = gpio.PWM(36, 50)
    pwm.start(7.9)
    pwm_31 = gpio.PWM(31, 50)
    pwm_33 = gpio.PWM(33, 50)
    pwm_35 = gpio.PWM(35, 50)
    pwm_37 = gpio.PWM(37, 50)
    pwm_val = 100

    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)

    counter_r = counter_l = 0
    tick_r = tick_l = 0
    ticks_r = ticks_l = []

    pwm_33.start(pwm_val)
    pwm_37.start(pwm_val)

    Kp = -2.1
# 4.7 Left
# 4.6 Anticlockwise
    while counter_r < 90 * 4.6:
    # while True:
        data = ser.readline()
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
        pwm_37.ChangeDutyCycle(val)

    if os.path.exists("encoder_right.txt"):
        os.remove("encoder_right.txt")
    else:
        print("The file does not exist")

    if os.path.exists("encoder_left.txt"):
        os.remove("encoder_left.txt")
    else:
        print("The file does not exist")

    f = open("encoder_right.txt", 'w')
    for i in ticks_r:
        f.write(str(i))
        f.write('\n')
    f.close()

    f = open("encoder_left.txt", 'w')
    for i in ticks_l:
        f.write(str(i))
        f.write('\n')
    f.close()
    
    f = open("plot.txt", "w")
    f.write(str(angle))
    f.write('\n')
    f.close

    pwm_33.stop()
    pwm_37.stop()
    pwm.stop()
    gpio.cleanup()

    print(f"Right encoder counts {counter_r} & Left encoder counts {counter_l}")
except:
    pwm_33.stop()
    pwm_37.stop()
    pwm.stop()
    gpio.cleanup()

    if os.path.exists("encoder_right.txt"):
        os.remove("encoder_right.txt")
    else:
        print("The file does not exist")

    if os.path.exists("encoder_left.txt"):
        os.remove("encoder_left.txt")
    else:
        print("The file does not exist")

    f = open("encoder_right.txt", 'w')
    for i in ticks_r:
        f.write(str(i))
        f.write('\n')
    f.close()

    f = open("encoder_left.txt", 'w')
    for i in ticks_l:
        f.write(str(i))
        f.write('\n')
    f.close()

    print(counter_l, counter_r)
    actual_count = (counter_l + counter_r)/2

    data = ser.readline()

    data = data.decode()
    angle = 360 - float(data.split(" ")[1][:-4])
    print(angle)
    
    try:
        per_angle = actual_count/angle
    except:
        pass

    angle_90 = per_angle * 90

    print(per_angle, angle_90)
