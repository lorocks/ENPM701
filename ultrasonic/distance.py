import RPi.GPIO as gpio
import time


trig = 16
echo = 18

# with this logic always remember to cleanup
gpio.setmode(gpio.BOARD)
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

gpio.output(trig, False)
time.sleep(0.01)

def dist():
    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)

    while gpio.input(echo) == 0:
        start = time.time()

    while gpio.input(echo) == 1:
        end = time.time()

    duration = end - start

    distance = duration * 17150
    distance = round(distance, 2)

    return distance

print(f"Distance {dist()} cm")
gpio.cleanup()

