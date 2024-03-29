import RPi.GPIO as g
import cv2
import time

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
    g.setmode(g.BOARD)
    g.setup(36, g.OUT)
    pwm = g.PWM(36, 50)

    open = 4
    close = 7.9
    current = open
    stepChange = (close - open) / 10

    pwm.start(current)


    frame_width = int(640)
    frame_height = int(480)

    size = (frame_width, frame_height)

    result = cv2.VideoWriter('video.avi',
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            5, size)
                            
    count = 0

    while True:
        frame = videostream.read()

        frame = cv2.flip(frame, -1)

        cv2.putText(frame,f"Duty cycle: {current}%",(20,20),cv2.FONT_HERSHEY_SIMPLEX ,1,(255,255,255),1,cv2.LINE_AA)


        result.write(frame)

        current += stepChange
        if current > close:
            current = close
            stepChange *= -1
        if current < close and current > open:
            pwm.ChangeDutyCycle(current)
            time.sleep(0.5)
            
        if current < open:
            current = open
            count += 1
            
        if count >= 5:
            break
        
        # Press key q to stop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pwm.stop()
    g.cleanup()
    videostream.stop()
    result.release()

except Exception as error:
    print("Error aborting!", error)
    pwm.stop()
    g.cleanup()
    videostream.stop()
    result.release()
