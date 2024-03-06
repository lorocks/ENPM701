import RPi.GPIO as gpio
import time
import cv2

# with this logic always remember to cleanup
trig = 16
echo = 18

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

distances = []

for i in range(10):
    distances.append(dist())

cap = cv2.VideoCapture(0)

ret, frame = cap.read()

if ret:
    frame = cv2.flip(frame, -1)
    cv2.putText(frame, f'Distance: {sum(distances)/len(distances)} cm', (30, 50), cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 0), 2,
                cv2.LINE_AA)
    
    cv2.imwrite("ultrasonic_dist.jpg", frame)

else:
    print("No camera connected")

cap.release()
