import RPi.GPIO as gpio
import time
import cv2

from threading import Thread

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

# Initialize the webcam
videostream = VideoStream().start()
time.sleep(1)

try:
    trig = 16
    echo = 18
    servo = 36

    open = 5
    close = 7.9

    gpio.setmode(gpio.BOARD)
    gpio.setup(servo,gpio.OUT)
    gpio.setup(trig,gpio.OUT)
    gpio.setup(echo,gpio.IN)

    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT)

    pwm = gpio.PWM(servo,50)
    pwm.start(close)
   
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
        
    def gameover():
        gpio.output(31,False)
        gpio.output(33,False)
        gpio.output(35,False)
        gpio.output(37,False)
        time.sleep(0.5)
    
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
        
    def forward():
        
        time.sleep(1)
        

    # Forward until reaching wall
    def movetill(encoder_count, u_dist):
        counter_r = counter_l = 0
        tick_r = tick_l = 0

        dist = wall_dist()

        gpio.output(31,True)
        gpio.output(33,False)
        gpio.output(35,False)
        gpio.output(37,True)

        while dist > u_dist:
            # if gpio.input(12) != tick_r:
            #     counter_r += 1
            #     tick_r = gpio.input(12)

            # if gpio.input(7) != tick_l:
            #     counter_l += 1
            #     tick_l = gpio.input(7)
            
            dist = wall_dist()
            print(dist)

        gameover()

    def reverse():
        gpio.output(31,False)
        gpio.output(33,True)
        gpio.output(35,True)
        gpio.output(37,False)
        time.sleep(1)
        gameover()

    def right():
        gpio.output(31,True)        
        gpio.output(33,False)
        gpio.output(35,True)
        gpio.output(37,False)
        time.sleep(1)
        gameover()

    def left():
        gpio.output(31,False)
        gpio.output(33,True)
        gpio.output(35,False)
        gpio.output(37,True)
        time.sleep(1)
        gameover()


    frame_width = int(640)
    frame_height = int(480)

    size = (frame_width, frame_height)

    result = cv2.VideoWriter('other_video.avi',
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            5, size)

    counter = 0

    while True:
        frame = videostream.read()

        image = cv2.flip(frame,-1)

        distance = dist()

        cv2.putText(image, f"Distance: {distance}",(30,30),cv2.FONT_HERSHEY_SIMPLEX ,1,(0, 0, 0),1,cv2.LINE_AA)

        result.write(image)

        if counter%2 == 0:
            key_press = input("Enter keyboard input:")
            if(key_press.lower()=='w'):
                movetill(100, 5)
                
            elif(key_press.lower()=='s'):
                reverse()
            elif(key_press.lower()=='a'):
                left()
            elif(key_press.lower()=='d'):
                right()
            if key_press == 'o':
                pwm.ChangeDutyCycle(open)
                time.sleep(0.2)
            elif key_press == 'c':
                pwm.ChangeDutyCycle(close - 0.5)
                time.sleep(0.2)
            if key_press == 'x':
                pwm.stop()
                gpio.cleanup()
                break

        counter+=1

    result.release()
    cv2.destroyAllWindows()
    videostream.stop()
    
except Exception as error:
    print("Error aborting!", error)
    pwm.stop()
    gpio.cleanup()
    result.release()
    cv2.destroyAllWindows()
    videostream.stop()
