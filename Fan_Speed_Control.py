import cv2
import time
import numpy as np
import Hand_Tracking_module as htm
import RPi.GPIO as GPIO
import math

# Setup GPIO for LED PWM control
LED_PIN = 18  # PWM Pin on Raspberry Pi
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
pwm = GPIO.PWM(LED_PIN, 1000)  # 1 kHz frequency
pwm.start(0)  # Start with 0% duty cycle (off)

# defining the camera size
w, h = 1280,720

def calculate_angle(a, b, c):
    """Calculate the angle between three points (landmarks)"""
    a = np.array(a)  # Index finger base
    b = np.array(b)  # Index finger middle
    c = np.array(c)  # Index finger tip

    # Calculate angle using cosine rule
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return np.degrees(angle)  # Convert to degrees

#defining the htm object
Detector=htm.HandDetector(detectionCon=0.7)

cap=cv2.VideoCapture(0)
cap.set(3,w)
cap.set(4,h)
p_time=0

while True:
    SUCCESS, img=cap.read()
    img=Detector.FindHands(img)
    lmlist=Detector.FindPosition(img,draw=False)
    # print(lmlist)

    if len(lmlist)!=0:
        #print(lmlist[4],lmlist[8])

        # Get landmarks for index finger
        index_base = lmlist[8]  # MCP (Base)
        index_middle = lmlist[7]  # PIP (Middle)
        index_tip = lmlist[6]  # TIP



         # cv2.circle(img, (x1, y1), 15, (69, 0, 259), cv2.FILLED)
        # cv2.circle(img, (x2, y2), 15, (69, 0, 259), cv2.FILLED)
        # cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        # cv2.circle(img, (cx, cy), 15, (69, 0, 259), cv2.FILLED)

        # Calculate the rotation angle
        angle = calculate_angle(index_base, index_middle, index_tip)

        # Map angle (0°-180°) to brightness (0-100%)
        brightness = np.clip(int((angle / 180) * 100), 0, 100)
        pwm.ChangeDutyCycle(brightness)

        # Display brightness level
        cv2.putText(img, f'Brightness: {brightness}%', (1000, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    c_time = time.time()
    fps = 1 / (c_time - p_time)
    p_time = c_time

    cv2.putText(img, f'FPS:{int(fps)}', (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Img", img)
    cv2.waitKey(1)