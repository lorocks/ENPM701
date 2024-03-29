import RPi.GPIO as g
import cv2
import time

try:
    g.setmode(g.BOARD)
    g.setup(36, g.OUT)
    pwm = g.PWM(36, 50)

    open = 4
    close = 7.9
    current = open
    stepChange = (close - open) / 10

    pwm.start(current)

    cap = cv2.VideoCapture(0)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    size = (frame_width, frame_height)

    result = cv2.VideoWriter('video.avi',
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            10, size)
                            
    count = 0

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        frame = cv2.flip(frame, -1)

        cv2.putText(frame,f"Duty cycle: {current}%",(20,20),cv2.FONT_HERSHEY_SIMPLEX ,1,(255,255,255),1,cv2.LINE_AA)

        cv2.imshow("Frame", frame)

        result.write(frame)

        current += stepChange
        if current > close:
            current = close
            stepChange *= -1
        if current < close and current > open:
            pwm.ChangeDutyCycle(current)
            
        if current < open:
            count += 1
            
        if count >= 15:
            break
        
        # Press key q to stop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pwm.stop()
    g.cleanup()
    cap.release()
    result.release()

except:
    print("Error aborting!")
    pwm.stop()
    g.cleanup()
    cap.release()
    result.release()
