#!/usr/local/bin/python2.7
import argparse as ap
import cv2
import numpy as np
import os
from sklearn.svm import LinearSVC
from sklearn.externals import joblib
from scipy.cluster.vq import vq, kmeans, whiten
from sklearn.preprocessing import StandardScaler
import time

def resize_image(image_file):
    img = cv2.imread(image_file)
    height,width = img.shape[:2]
    aspect_ratio = float(width)/height

    if width > 640:
        width_new = 640
        height_new = int(width_new / aspect_ratio)
        img = cv2.resize(img, (width_new, height_new), interpolation = cv2.INTER_CUBIC)
    return img

train_directory = "dataset/train"

class_names = []
image_files = []
image_labels = []
class_id = 0
for root, dirs, files in os.walk(train_directory, topdown=False):
    for d in dirs:
        if not d.startswith('.'):
            class_names.append(d)
        num_files = 0
        for f in os.listdir(os.path.join(root,d)):
            if f.endswith('.jpg') or f.endswith('.JPG') or f.endswith('.png') or f.endswith('.jpeg') or f.endswith('.JPEG'):
                image_files.append(root + '/' + d + '/' + f)
                num_files += 1
        image_labels += [class_id] * num_files
        class_id += 1

print "Class Names:", class_names
print "Number of Image Files", len(image_files)

# Create SIFT object
sift = cv2.SIFT()

# List where all the descriptors are stored
print "Creating descriptor list"
start = time.time()

descriptor_list = []
image_files_to_remove = []
descriptors = np.zeros((1,128))
for image_file in image_files:
    image = resize_image(image_file)
    kp, des = sift.detectAndCompute(image, None)
    descriptor_list.append((image_file, des))
    descriptors = np.vstack((descriptors, des))

    ## in order for this check to work, will also need to remove elements from image_labels
    # if des is not None:
    #     descriptor_list.append((image_file, des))
    #     descriptors = np.vstack((descriptors, des))
    # else: # occurs if there are no discernable SIFT features in the image
    #     print "des is empty, removing image"
    #     print image_file
    #     image_files_to_remove.append(image_file)

# for image_file in image_files_to_remove:
#     index = image_files.index(image_file)
#     del image_labels[index]
#     image_files.remove(image_file)

descriptors = np.delete(descriptors, 0, 0)

print len(image_files)
print len(descriptor_list)

end = time.time()
print "Creating descriptor list time:",(end - start)

# Perform k-means clustering
print "K-Means Clustering"
start = time.time()

descriptors = whiten(descriptors)

k = 100
vocabulary, variance = kmeans(descriptors, k, 1)

end = time.time()
print "K-means clustering time:",(end - start)

# Calculate the histogram of features
print "Creating histogram of features"
start = time.time()

features = np.zeros((len(image_files), k), "float32")
for i in xrange(len(image_files)):
    words, distance = vq(whiten(descriptor_list[i][1]), vocabulary)
    for w in words:
        if w >= 0 and w < 100:
            features[i][w] += 1

end = time.time()
print "Creating histogram of features time:",(end - start)

print "SVM setup"
start = time.time()

# Scaling the words
std_slr = StandardScaler().fit(features)
features = std_slr.transform(features)

end = time.time()
print "SVM setup time:",(end - start)

# Train the Linear SVM
print "SVM training"
start = time.time()

classifier = LinearSVC()
classifier.fit(features, np.array(image_labels))

end = time.time()
print "SVM training time:",(end - start)

# Save the SVM
joblib.dump((classifier, class_names, std_slr, k, vocabulary), "trained_variables.pkl", compress=3)
