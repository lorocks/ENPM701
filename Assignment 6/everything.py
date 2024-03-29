import RPi.GPIO as gpio
import time
import cv2

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
        
    def reverse():
        gpio.output(31,True)
        gpio.output(33,False)
        gpio.output(35,False)
        gpio.output(37,True)
        time.sleep(1)
        gameover()

    def forward():
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


    cap = cv2.VideoCapture(0)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    size = (frame_width, frame_height)

    result = cv2.VideoWriter('other_video.avi',
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            10, size)

    counter = 0

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        image = cv2.flip(frame,-1)

        distance = dist()

        cv2.putText(image, f"Distance: {distance}",(100,100),cv2.FONT_HERSHEY_SIMPLEX ,1,(255,255,255),1,cv2.LINE_AA)

        result.write(image)

        if counter%2 == 0:
            key_press = input("Enter keyboard input:")
            if(key_press.lower()=='w'):
                forward()
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
                pwm.ChangeDutyCycle(close)
                time.sleep(0.2)
            if key_press == 'x':
                pwm.stop()
                gpio.cleanup()
                break

        counter+=1

    result.release()
    cv2.destroyAllWindows()
except Exception as error:
    print("Error aborting!", error)
    pwm.stop()
    gpio.cleanup()
    result.release()
    cv2.destroyAllWindows()
