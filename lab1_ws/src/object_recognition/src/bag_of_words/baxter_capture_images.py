#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class baxter_capture_images:
    def __init__(self):
        self.bridge = CvBridge()
        # baxter camera Subscriber
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)

        self.directory = 'dataset/train/eraser'
        print "DIRECTORY IS: ",self.directory
        if not os.path.exists(self.directory):
            os.mkdir(self.directory)

        self.counter = 0

    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print("==[CAMERA MANAGER]==",e)

        scale = 0.75
        frame = (frame*scale).astype(np.uint8)

        if cv2.waitKey(1) & 0xFF == ord('c'):
            image_path = self.directory + '/' + str(self.counter) + '.jpg'
            cv2.imwrite(image_path, frame)
            print "Capturing image",image_path
            self.counter += 1

        cv2.imshow("frame",frame)
        cv2.waitKey(3)

def main(args):
    rospy.init_node('baxter_capture_images', anonymous=True)
    ic = baxter_capture_images()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print "OpenCV Version:",cv2.__version__
    main(sys.argv)
