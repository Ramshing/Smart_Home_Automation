import cv2
import time
import numpy as np
import Hand_Tracking_module as htm
import RPi.GPIO as GPIO
import math
import serial
from cvzone.SerialModule import SerialObject

# arduino = SerialObject(portNo='COM13')

arduino = serial.Serial('COM10', 9600, timeout=1)


# Setup GPIO for LED PWM control
LED_PIN = 18  # PWM Pin on arduino
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
pwm = GPIO.PWM(LED_PIN, 1000)  # 1 kHz frequency
pwm.start(0)  # Start with 0% duty cycle (off)

# defining the camera size
width_cam, height_cam = 1280,720

#defining the htm object
Detector=htm.HandDetector(detectionCon=0.7)

cap=cv2.VideoCapture(0)
cap.set(3,width_cam)
cap.set(4,height_cam)
p_time=0

while True:
    SUCCESS, img=cap.read()
    img=Detector.FindHands(img)
    lmlist=Detector.FindPosition(img,draw=False)
    # print(lmlist)

    if len(lmlist)!=0:
        #print(lmlist[4],lmlist[8])

        # To get landmark of the fingers
        x1,y1 = lmlist[4][1], lmlist[4][2]
        x2,y2 = lmlist[8][1], lmlist[8][2]
        cx,cy=(x1+x2)//2,(y1+y2)//2   # To get center point between fingers

        cv2.circle(img, (x1, y1), 15, (69, 0, 259), cv2.FILLED)
        cv2.circle(img, (x2, y2), 15, (69, 0, 259), cv2.FILLED)
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.circle(img, (cx, cy), 15, (69, 0, 259), cv2.FILLED)

        # To find length between two fingers
        length = math.hypot(x2 - x1, y2 - y1)
        print(length)

        # Map distance to LED brightness (0-100%)
        brightness = np.clip(int(length / 3), 0, 100)  # Adjust scaling factor
        # pwm.ChangeDutyCycle(brightness)
        arduino.write(f"{brightness}\n".encode())

        # Display brightness level
        cv2.putText(img, f'Brightness: {brightness}%', (1000, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    c_time = time.time()
    fps = 1 / (c_time - p_time)
    p_time = c_time

    cv2.putText(img, f'FPS:{int(fps)}', (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Img", img)
    cv2.waitKey(1)