#!/usr/local/bin/python2.7
import argparse as ap
import cv2
import numpy as np
import os

cap = cv2.VideoCapture(0)

directory = 'dataset/train/screwdriver'
print "DIRECTORY IS: ",directory
if not os.path.exists(directory):
    os.mkdir(directory)

counter = 0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if cv2.waitKey(25) & 0xFF == ord('c'):
        image_path = directory + '/' + str(counter) + '.jpg'
        cv2.imwrite(image_path, frame)
        print "Capturing image",image_path
        counter = counter + 1


    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
