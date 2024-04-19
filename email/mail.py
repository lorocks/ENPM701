import os
import datetime
import smtplib
import email
import cv2
import base64

from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

time_now = datetime.datetime.now().strftime('%Y%m%d%H%M%S')

smtpUser = 'ENPM701.lorocks@gmail.com'
smtpPass = 'jtesrcwsaygtxubj'

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()

    image = cv2.flip(frame, -1)

    break

# buffer = cv2.imencode('.jpg', image)[1].tobytes()

# to = 'lorocks@umd.edu'
# fromAdd = smtpUser
# subject = 'Test image'
# msg = MIMEMultipart()
# msg['Subject'] = subject
# msg['From'] = fromAdd
# msg['To'] = to
# msg.preamble = 'testing 123!'

# body = MIMEText('testing aaaaa')
# msg.attach(body)


# img = MIMEImage(buffer)
# msg.attach(img)


# s = smtplib.SMTP('smtp.gmail.com', 587)

# s.ehlo()
# s.starttls()
# s.ehlo()

# s.login(smtpUser, smtpPass)
# s.sendmail(fromAdd, to, msg.as_string())
# s.quit()

# print("Email send ehe")



def send_email(image):
    time_now = datetime.datetime.now().strftime('%Y%m%d%H%M%S')

    smtpUser = 'ENPM701.lorocks@gmail.com'
    smtpPass = 'jtesrcwsaygtxubj'

    buffer = cv2.imencode('.jpg', image)[1].tobytes()


    to = 'ENPM809TS19@gmail.com'
    fromAdd = smtpUser
    cc = 'lorocks@umd.edu'
    msg = MIMEMultipart()
    msg['Subject'] = f'ENPM701-HW9-BlockRetrieved-{time_now}-Lowell_Lobo-lorocks'
    msg['From'] = fromAdd
    msg['To'] = to
    msg['Cc'] = cc
    msg.preamble = 'Image from RPi'

    body = MIMEText(f'Image Found at {time_now}')
    msg.attach(body)

    img = MIMEImage(buffer)
    msg.attach(img)

    s = smtplib.SMTP('smtp.gmail.com', 587)

    s.ehlo()
    s.starttls()
    s.ehlo()

    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, to.split(",") + cc.split(","), msg.as_string())
    s.quit()

    print("Email send")


send_email(image)