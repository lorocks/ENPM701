# change pwm based on encoder counts
# pwm_val = actual_val + correction
# correction = Kp * (counter_r - counter_l)

# Check if 31 left or right, and 12 left or right

import RPi.GPIO as gpio
import time

try:
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(37,gpio.OUT) #IN4

    pwm_31 = gpio.PWM(31, 50)
    pwm_37 = gpio.PWM(37, 50)
    pwm_val = 25
    Kp = 0.7

    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)

    counter_r = counter_l = 0
    tick_r = tick_l = 0
    ticks_r = ticks_l = []

    pwm_31.start(pwm_val)
    pwm_37.start(pwm_val)

    while counter_r < 50:
        ticks_r.append(gpio.input(12))
        ticks_l.append(gpio.input(7))

        if gpio.input(12) != tick_r:
            counter_r += 1
            tick_r = gpio.input(12)

        if gpio.input(7) != tick_l:
            counter_l += 1
            tick_l = gpio.input(7)
        
        error = counter_r - counter_l
        if error > 0:
            pwm_31.ChangeDutyCycle(pwm_val - (Kp * error))
            pwm_37.ChangeDutyCycle(pwm_val)
        elif error < 0:
            pwm_37.ChangeDutyCycle(pwm_val - (Kp * error))
            pwm_31.ChangeDutyCycle(pwm_val)

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

    pwm_31.stop()
    pwm_37.stop()
    gpio.cleanup()

    print(f"Right encoder counts {counter_r} & Left encoder counts {counter_l}")
except:
    pwm_31.stop()
    pwm_37.stop()
    gpio.cleanup()