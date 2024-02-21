import cv2
import imutils
import time
import numpy as np
import os

cap = cv2.VideoCapture(0)


time_vals = []

# Define the lower and upper color thresholds
lower = np.array([40, 60, 50])
upper = np.array([95, 255, 255])
lower = np.array([50, 70, 100])

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

size = (frame_width, frame_height)

result = cv2.VideoWriter('video.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

count = 0

while cap.isOpened():
    count += 1
    if count == 100:
       print("Frame number 100 reached")
    start = time.time()
    # Read frame from the video
    ret, frame = cap.read()

    if not ret:
        break

    # Update object localizer
    # frame = cv2.flip(frame, -1)

    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask based on the color thresholds
    mask = cv2.inRange(hsv_image, lower, upper)

    # Find contour
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding box
    for contour in contours:
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        # x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)

    cv2.imshow("Video Stream", frame)
    # cv2.imshow("hsv", mask)

    result.write(frame)

    time1 = time.time() - start
    time_vals.append(time1)

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
cap.release()

print("Writing data to file")

if os.path.exists("hw3data.txt"):
  os.remove("hw3data.txt")
else:
  print("The file does not exist")

f = open('hw3data.txt', 'a')
for val in time_vals:
  f.write(str(val))
  f.write('\n')
f.close()
