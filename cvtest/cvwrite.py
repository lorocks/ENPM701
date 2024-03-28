import cv2
import time

video = cv2.VideoCapture(0)

frame_width = int(video.get(3))
frame_height = int(video.get(4))

size = (frame_width, frame_height)

result = cv2.VideoWriter('video.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)
start = time.time()
frame_rate_calc = 1
while(True):
    t1 = cv2.getTickCount()
    ret, frame = video.read()

    if ret:
        # cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
        
        result.write(frame)
        cv2.imshow('Frame', frame)

        t2 = cv2.getTickCount()
        time1 = (t2 - t1) / cv2.getTickFrequency()
        frame_rate_calc = 1 / time1
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if time.time() - start >35:
              break

    else:
        break

video.release()
result.release()
cv2.destroyAllWindows()

print("The video was successfully saved")
