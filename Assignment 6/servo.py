import RPi.GPIO as g
import cv2

try:
    g.setmode(g.BOARD)
    g.setup(36, g.OUT)
    pwm = g.PWM(36, 50)

    open = 3.1
    close = 6.9

    pwm.start(close)

    cap = cv2.VideoCapture(0)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    size = (frame_width, frame_height)

    result = cv2.VideoWriter('video.avi',
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            10, size)
    
    current = close
    stepChange = -0.5

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        cv2.putText(frame,f"Duty cycle: {current}%",(20,20),cv2.FONT_HERSHEY_SIMPLEX ,1,(255,255,255),1,cv2.LINE_AA)

        result.write(frame)

        current += stepChange
        if current < open:
            current = open
            stepChange *= -1
        if current > close:
            current = close
            stepChange *= -1
            break
        
        pwm.ChangeDutyCycle(current)
        
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