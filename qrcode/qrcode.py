import qrcode

code = qrcode.make("ENPM 701")
code.save("qrcodeenpm.jpg")

print("QR Code generated")