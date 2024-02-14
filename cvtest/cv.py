import cv2
import time

cap = cv2.VideoCapture(0)

record = cv2.VideoWriter('video.avi',cv2.VideoWriter_fourcc(*"XVID"), 2, (1280,720))

start = time.time()

frame_rate_calc = 1

while cap.isOpened():
	t1 = cv2.getTickCount()
	ret, frame = cap.read()
	if not ret:
		break

	cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 0), 2,
                cv2.LINE_AA)
	cv2.imshow("Frame", frame)

	record.write(frame)

	t2 = cv2.getTickCount()
	time1 = (t2 - t1) / cv2.getTickFrequency()
	frame_rate_calc = 1 / time1
    
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

	if time.time() - start > 5:
		  break

cv2.destroyAllWindows()
cap.release()
