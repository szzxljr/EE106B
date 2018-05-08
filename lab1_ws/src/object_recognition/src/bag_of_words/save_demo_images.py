#!/usr/local/bin/python2.7
import cv2
import numpy as np
from sklearn.externals import joblib
from scipy.cluster.vq import vq, kmeans, whiten

# Load the classifier, class names, scaler, number of clusters and vocabulary
classifier, class_names, std_slr, k, vocabulary = joblib.load("dataset6.pkl")
# classifier, class_names, std_slr, k, vocabulary = joblib.load("trained_variables.pkl")

cap = cv2.VideoCapture(0)

# Create SIFT object
sift = cv2.SIFT()

# Variables for filtering results
confidences = [0,0,0,0,0]
counter = 0

needResizing = False

imageName = 'hand_points'

fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.mp4',fourcc, 5.0, (640,480))

while(True):
    # Used for filtering results
    if counter > 4:
        counter = 0

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Split frame into left and right halves
    # left_half = frame[0:480, 0:640/2]
    # right_half = frame[0:480, 640/2:640]

    # Create list of left and right images
    # frames = [left_half, right_half]
    frames = [frame]

    # Classify object in each half
    for index,f in enumerate(frames):
        # List where all the descriptors are stored
        descriptor_list = []
        kp, des = sift.detectAndCompute(f, None)
        frames[index] = cv2.drawKeypoints(f, kp, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) # draw SIFT points
        descriptor_list.append(('curFrame', des))

        # Check to make sure descriptor_list has elements
        if descriptor_list[0][1] is not None and len(kp) > 15:
            test_features = np.zeros((1, k), "float32")
            words, distance = vq(whiten(descriptor_list[0][1]), vocabulary)
            for w in words:
                if w >= 0 and w < 100:
                    test_features[0][w] += 1

            # Scale the features
            test_features = std_slr.transform(test_features)

            # predictions based on classifier (more than 2)
            predictions = [class_names[i] for i in classifier.predict(test_features)]

            # Add label to half of the image
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frames[index], predictions[0], (225,100), font, 1, (255,255,255), 2)
            # if index == 0: # if it's the left half of the image
            #     cv2.putText(frames[index], predictions[0], (40,40), font, 1, (255,255,255), 2)
            # elif index == 1: # if it's the right half of the image
            #     cv2.putText(frames[index], predictions[0], (360,40), font, 1, (255,255,255), 2)

    # Add a dividing line down the middle of the frame
    # cv2.line(frame, (318,0), (318,480), (255,0,0), 4)

    # Resize image to fit monitor (if monitor is attached)
    # frame = cv2.resize(frame, (1920, 1080), interpolation = cv2.INTER_CUBIC)

    if cv2.waitKey(1) & 0xFF == ord('c'):
        image_path = imageName + '.jpg'
        cv2.imwrite(image_path, frames[0])
        print "Capturing image",image_path

    # Display the resulting frame
    for i,f in enumerate(frames):
        # Resize image to fit monitor (if monitor is attached)
        if i == 0:
            if needResizing:
                frames[i] = cv2.resize(f, (1920, 1080), interpolation = cv2.INTER_CUBIC)
            # out.write(frames[i])
            cv2.imshow('f'+str(i), frames[i])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop capturing and close
cap.release()
cv2.destroyAllWindows()
