import numpy as np
from math import sin, cos, atan2
from geometry_msgs.msg._Point import Point
from collections import deque
import argparse
# import imutils
import cv2
import rospy
import time

def vec(*args):
	if len(args) == 1:
	  	if type(args[0]) == tuple:
			return np.array(args[0])
		elif type(args[0]) == Point:
			return np.array((args[0].x, args[0].y, args[0].z))
		else:
			return np.array(args)
	else:
		return np.array(args)

def hat(v):
	if v.shape == (3, 1) or v.shape == (3,):
		return np.array([
				[0, -v[2], v[1]],
				[v[2], 0, -v[0]],
				[-v[1], v[0], 0]
			])
	elif v.shape == (6, 1) or v.shape == (6,):
		return np.array([
				[0, -v[5], v[4], v[0]],
				[v[5], 0, -v[3], v[1]],
				[-v[4], v[3], 0, v[2]],
				[0, 0, 0, 0]
			])
	else:
		raise ValueError

def adj(g):
	# print g
	if g.shape == (4, 4):
		R = g[0:3,0:3]
		p = g[0:3,3]
		result = np.zeros((6, 6))
		result[0:3,0:3] = R
		result[0:3,3:6] = hat(p) * R
		result[3:6,3:6] = R
		return result
	else:
		raise ValueError

def twist_from_tf(g):
	return vec(g[0,2], g[1,2], atan2(g[1,0], g[0,0]))

def rotation2d(theta):
	return np.array([
			[cos(theta), -sin(theta)],
			[sin(theta), cos(theta)]
		])

def rigid(twist):
	return np.array([
			[cos(twist[2]), -sin(twist[2]), twist[0]],
			[sin(twist[2]), cos(twist[2]), twist[1]],
			[0, 0, 1]
		])

def rotationX(theta):

	return np.array([[1,0,0],
			[0,cos(theta), -sin(theta)],
			[0,sin(theta), cos(theta)]
		])

def rotationY(theta):
	return np.array([[cos(theta),0,sin(theta)],
			[0,1,0],
			[sin(theta),0, -cos(theta)]
		])

def rotationZ(theta):
	return np.array([[0,0,1],
			[cos(theta), -sin(theta),0],
			[sin(theta), cos(theta),0]
		])

def rotation(x,y,z):
	return rotationX(x)*rotationY(y)*rotationZ(z)


def ball_tracking():
	# ap = argparse.ArgumentParser()
	# ap.add_argument("-v", "--video", help="path to the (optional) video file")
	# args = vars(ap.parse_args())
	t0 = time.time()
	greenLower = (25, 51, 30)
	greenUpper = (100, 255, 178)
	x_list = []
	y_list = []
	r_list = []
	# if not args.get("video", False):
	# else:
	#     camera = cv2.VideoCapture(args["video"])
	global image
	# raw_input()
	# start = time.time()
	camera = cv2.VideoCapture(0)
	while True:
	    (grabbed, frame) = camera.read()
	    # if args.get("video") and not grabbed:
	    #     break

	    # frame = imutils.resize(frame, width=600)
	    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	    mask = cv2.inRange(hsv, greenLower, greenUpper)
	    mask = cv2.erode(mask, None, iterations=2)
	    mask = cv2.dilate(mask, None, iterations=2)

	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	    center = None

	    if len(cnts) > 0:
	        c = max(cnts, key=cv2.contourArea)
	        ((x, y), radius) = cv2.minEnclosingCircle(c)
	        t1 = time.time()
	        if radius > 15 and radius < 90 and (t1 - t0) > 1:
	            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	            # print('x:',x)
	            # print('y',y)
	            print('radius',radius)
	            x_list.append(x)
	            y_list.append(y)
	            r_list.append(radius)
	    cv2.imshow("Frame", frame)
	    # cv2.imshow("Mask", mask)

	    key = cv2.waitKey(1) & 0xFF
	    if key == ord("l"):
	        break


	# end = time.time()
	# print("time", end - start) 

	camera.release()
	cv2.destroyAllWindows()
	# x_list.pop(0)
	# y_list.pop(0)
	# x_list.pop(0)
	# y_list.pop(0)
	# x_list.pop(0)
	# y_list.pop(0)
	for i in range(len(x_list)):
	    cv2.circle(frame,(int(x_list[i]),int(y_list[i])),int(r_list[i]),(0,0,255),1)
	cv2.imshow("Frame", frame)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	inputlist = [x_list[0],y_list[0],x_list[1],y_list[1],x_list[2],y_list[2],x_list[3],y_list[3],x_list[4],y_list[4]]

	return inputlist

def getEndPointPosition(limb):
    pose = limb.endpoint_pose()
    poselist = list(tuple(pose['position']) + tuple(pose['orientation']))
    return poselist # return value is a 7x 'list'
    
def lookup_tag(tag_number):
    
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)
    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
    return (tag_pos + tag_rot)  
# while True:
# 	ball_tracking()