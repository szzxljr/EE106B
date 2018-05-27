from collections import deque
import numpy as np
import argparse
# import imutils
import cv2
import rospy
import time

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
args = vars(ap.parse_args())

# greenLower = (29, 86, 6)
# greenUpper = (64, 255, 255)
greenLower = (25, 51, 30)
greenUpper = (100, 255, 178)
x_list = []
y_list = []
r_list = []
time_list = []
if not args.get("video", False):
    camera = cv2.VideoCapture(0)
else:
    camera = cv2.VideoCapture(args["video"])
global image


# raw_input()
# start = time.time()

while True:
    (grabbed, frame) = camera.read()

    if args.get("video") and not grabbed:
        break

    # frame = imutils.resize(frame, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    cv2.circle(frame,(320,240),25,(0,255,255),2)
    cv2.circle(frame,(420,240),25,(0,255,255),2)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 15 and radius < 90:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            t = time.time()
            print('x:',x)
            print('y',y)
            # print('radius',radius)
            x_list.append(x)
            y_list.append(y)
            r_list.append(radius)
            time_list.append(t)
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("c"):
        break


# end = time.time()
# print("time", end - start) 

camera.release()
cv2.destroyAllWindows()
x_list.pop(0)
y_list.pop(0)
r_list.pop(0)
x_list.pop(0)
y_list.pop(0)
r_list.pop(0)
time_list.pop(0)
time_list.pop(0)
for i in range(len(x_list)):
    cv2.circle(frame,(int(x_list[i]),int(y_list[i])),int(r_list[i]),(0,0,255),1)
cv2.imshow("Frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
print('number of points',len(x_list))
print('total time',time_list[-1] - time_list[0])

# camera.release()
# cv2.destroyAllWindows()

