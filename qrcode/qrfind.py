import cv2
import os

# start_cam = "sudo modprobe bcm2835-v4l2"
# os.system(start_cam)

cap = cv2.VideoCapture(0)

detect = cv2.QRCodeDetector()

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    data, bbox, _ = detect.detectAndDecode(frame)

    if bbox is not None:
        bbox = bbox.astype(int)
        cv2.rectangle(frame, bbox[0][0], bbox[0][2], (255, 0, 0), 2)
        cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    if data:
        print(data)

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


    