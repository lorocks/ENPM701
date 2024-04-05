import RPi.GPIO as gpio
import time
import serial

# Better to use imu to get it :O

try:
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33, gpio.OUT) 
    gpio.setup(35, gpio.OUT)
    gpio.setup(37,gpio.OUT) #IN4

    pwm_31 = gpio.PWM(31, 50)
    # pwm_33 = gpio.PWM(33, 50)
    pwm_35 = gpio.PWM(35, 50)
    # pwm_37 = gpio.PWM(37, 50)
    pwm_val = 50

    Kp = -1.9

    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)

    counter_r = counter_l = 0
    tick_r = tick_l = 0
    ticks_r = ticks_l = []

    pwm_31.start(pwm_val)
    pwm_35.start(pwm_val)

    while True:
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
        pwm_35.ChangeDutyCycle(val)

    
except:
    pwm_31.stop()
    pwm_35.stop()

    ser = serial.Serial('/dev/ttyUSB0', 9600)

    count = 0

    printed = False
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

    # print(counter_l, counter_r)
    # actual_count = counter_l

    # if counter_l < counter_r:
    #     actual_count = counter_r
    
    # per_angle = data/actual_count

    # angle_90 = per_angle * 90

    # print(per_angle, angle_90)

    print(f"Right encoder counts {counter_r} & Left encoder counts {counter_l}")
    
    gpio.cleanup()