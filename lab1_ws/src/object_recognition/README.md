# Object Recognition with Baxter
MSR Winter Project focused on training Baxter to recognize and pickup objects

## Overview  
This repo contains code to train and test a three-object classifier, run this classifier on a Baxter robot, and instruct Baxter to pick up each object. More information about methodology can be found [here](https://apollack11.github.io/object-recognition.html).

## Code
### src/baxter_demo/
This directory contains the code relevant to the demo in this [video](https://vimeo.com/218413450).

**Nodes**  
`baxter_object_prediction.py`: Subscribes to Baxter's camera feed and segments the image into three sections. Within each of those sections, I use OpenCV to solve for the largest contour in the image and then run each image through the classifier to determine the name of the object. This node then publishes the name and location of all three objects in the image.  

`request_object.py`: Runs in a separate terminal which allows the use to choose and object they would like Baxter to pick up. If the object the user has selected is valid, this node then publishes a message with the name of the desired object.  

`pickup_object.py`: This node subscribes to request_object.py and baxter_object_prediction.py. It waits for a desired object to be requested and then moves Baxter's arm to the location of the desired object, picks up the object, and places it down again.

**Additional Scripts**  
`planning_node.py`: Interfaces with MoveIt! to perform inverse kinematics solving and path planning. Is used by pickup_object.py to move to the desired object location.

`grip_node.py`: Interfaces with the gripper at the end of Baxter's hand. Is used to grip the object once planning_node.py has moved to the desired object as instructed by pickup_object.py.

### src/bag_of_words/  
Contains code to train and test the classifier on a set of images. Also contains scripts to automatically download images from Google and capture additional images for testing.

**Scripts**  
`baxter_capture_images.py` and `capture_images.py`: These two scripts help automate the process of capturing images for training the classifier. The first script takes the video feed from Baxter's hand and allows the user to press 'c' to save a new image from the current frame of the camera. The second script does the same from a traditional webcam.  

`sift_live_predict`: Takes a video feed from a webcam, extracts SIFT features, and runs those features through the classifier to determine the identity of the object in the image. This script provides a live video feed of the object with a label naming that object.  

`multiple_object_predict.py`: Does the same thing as sift_live_predict.py but for multiple objects. In this case, it separates the image into three separate images, finds the largest contour in each image, and classifies each of three images. The result is a live video feed with each object outlined in a bounding box with a label of the object's name.

`sift_train_classifier.py`: Given a dataset of images (in the dataset directory), trains a classifier using the Bag of Words method to recognize each object. Automatically labels the images based on the name of the directory they're in.

`sift_predict_class.py`: Can take three separate arguments (-t, -i, or -s). -t allows the user to test on a directory of test images (separated into folders according to classification) and returns a list of classifications and a test accuracy based on the classifier trained in sift_train_classifier.py. -i allows the user to test the classifier on a single image. -s allows the user to test the classifier on a particular sub-folder (e.g. just screwdriver) and returns a list of classifications for each image in that folder.

`scrape_images.py`: A script to pull images from a Google image search. Useful for quickly gathering a set of images for training.

### src/surf_and_sift_testing/  
A directory containing images to test the results of extracting SIFT features.
