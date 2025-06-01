import time
import cv2
import mediapipe as mp

class HandDetector():
    def __init__(self,mode=False,maxHands=2,detectionCon=0.5,trackCon=0.5):
        self.mode=mode
        self.maxHands=maxHands
        self.detectionCon=detectionCon
        self.trackCon=trackCon

        self.mphands = mp.solutions.hands
        self.Hands = self.mphands.Hands()
        self.mpdraw = mp.solutions.drawing_utils


    def FindHands(self,img,draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # converting BGR image to RGB image
        self.result = self.Hands.process(imgRGB)
        #print(result.multi_hand_landmarks)

        # If multiple hands are there then fetch one by one and draw dots and connections
        if self.result.multi_hand_landmarks:
            for handlms in self.result.multi_hand_landmarks:
                if draw:
                    # drawing the connection line for all landmarks
                    self.mpdraw.draw_landmarks(img, handlms, self.mphands.HAND_CONNECTIONS)

        return img

    def FindPosition(self,img,HandNo=0,draw=True):
        lmlist=[]
        if self.result.multi_hand_landmarks:
            myhand=self.result.multi_hand_landmarks[HandNo]
            for id, lm in enumerate(myhand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id,cx,cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 0), cv2.FILLED)

        return lmlist


def main():
    # for calculating FPS
    p_time = 0
    c_time = 0
    cap = cv2.VideoCapture(0)
    detector=HandDetector()
    while True:
        SUCCESS, img = cap.read()  # Capture the video
        img=detector.FindHands(img)
        lmlist=detector.FindPosition(img)
        if len(lmlist)!=0:
            print((lmlist[4]))
        # calculating the FPS
        c_time = time.time()
        fps = 1 / (c_time - p_time)
        p_time = c_time

        # Displaying the FPS
        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_COMPLEX, 3, (255, 0, 0), 3)

        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__=="__main__":
    main()