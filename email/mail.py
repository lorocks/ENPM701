import os
import datetime
import smtplib
import email

from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

time_now = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + time_now + '.jpg'
os.system(command)

smtpUser = 'ENPM701.lorocks@gmail.com'
smtpPass = 'jtesrcwsaygtxubj'


to = 'lorocks@umd.edu'
fromAdd = smtpUser
subject = 'Test image'
msg = MIMEMultipart()
msg['Subject'] = subject
msg['From'] = fromAdd
msg['To'] = to
msg.preamble = 'testing 123!'

body = MIMEText('testing aaaaa')
msg.attach(body)

# fp = open(time_now + '.jpg','rb')
# img = MIMEImage(fp.read())
# fp.close()
# msg.attach(img)


s = smtplib.SMTP('smtp.gmail.com', 587)

s.ehlo()
s.starttls()
s.ehlo()

s.login(smtpUser, smtpPass)
s.sendmail(fromAdd, to, msg.as_string())
s.quit()

print("Email send ehe")
