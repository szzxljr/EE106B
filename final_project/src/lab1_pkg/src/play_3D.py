import numpy as np
from math import sin, cos, atan2
from geometry_msgs.msg._Point import Point
from collections import deque
import argparse
# import imutils
import cv2
import rospy
import time
import copy
import sys
import argparse

import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

# import IPython
import tf
import tf2_ros
import time
from utils import *
from baxter_pykdl import baxter_kinematics
import math
import scipy.io as sio
import os

from backprop import *
# import backprop
from sklearn.linear_model import Ridge

rospy.init_node('moveit_node')
arm = 'right'
limb = baxter_interface.Limb(arm)
kin = baxter_kinematics(arm)
r = rospy.Rate(1000)
# position{}
# print(cur_pos)
tSpan = 0.1 #in second
joint_names = limb.joint_names()
tar_velocity = {}
error = {}
error_old = {}
derror = {}
error_new = {}
cur_pos = limb.joint_angles()
discounttime = 2
discount = math.sqrt(0.05)
Kp = 50
Kv = 2
start_time = rospy.get_time() #in second
deltat = 0
joint_names = limb.joint_names()
# tar_position ={'right_s0': 0.002684466378799474, 'right_s1': -0.5518495884417776, 'right_w0': -0.006135923151541655, 'right_w1': 1.254412789293297, 'right_w2': 0.005368932757598948, 'right_e0': -0.0019174759848567672, 'right_e1': 0.7531845668517382}
tar_position = {}

def load_model(num_layers,path1,path2,path3,path4):
    model_W = []
    model_b = []
    model_ridge_WW = np.loadtxt(path3)
    model_ridge_W = np.reshape(model_ridge_WW,(model_ridge_WW.shape[0],-1))
    model_ridge_ii = np.loadtxt(path4)
    model_ridge_intercept = np.reshape(model_ridge_ii,(model_ridge_ii.shape[0],-1))
    for i in range(num_layers):
        WW = np.loadtxt(path1.format(i))
        model_W.append( np.reshape(WW,(WW.shape[0],-1)) )
        bb = np.loadtxt(path2.format(i))
        model_b.append( np.reshape(bb,(bb.shape[0],-1)) )
    return model_W , model_b, model_ridge_W, model_ridge_intercept

def ball_tracking():
	# ap = argparse.ArgumentParser()
	# ap.add_argument("-v", "--video", help="path to the (optional) video file")
	# args = vars(ap.parse_args())
	greenLower = (25, 51, 30)
	greenUpper = (100, 255, 178)
	x_list = []
	y_list = []
	r_list = []

	# if not args.get("video", False):
	camera = cv2.VideoCapture(0)
	# else:
	#     camera = cv2.VideoCapture(args["video"])
	global image


	# raw_input()
	# start = time.time()

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
	        
	        if radius > 15 and radius < 90:
	            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	            # print('x:',x)
	            # print('y',y)
	            print('radius',radius)
	            x_list.append(x)
	            y_list.append(y)
	            r_list.append(radius)

	    cv2.imshow("Frame", frame)
	    # cv2.imshow("Mask", mask)

	    # key = cv2.waitKey(1) & 0xFF
	    if len(x_list) == 7:
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
	x_list.pop(0)
	y_list.pop(0)
	r_list.pop(0)
	for i in range(len(x_list)):
	    cv2.circle(frame,(int(x_list[i]),int(y_list[i])),int(r_list[i]),(0,0,255),1)
	cv2.imshow("Frame", frame)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	inputlist = [x_list[0],y_list[0],r_list[0],x_list[1],y_list[1],r_list[1],x_list[2],y_list[2],r_list[2],x_list[3],y_list[3],r_list[3]]
	return inputlist

def getEndPointPosition(limb):
    pose = limb.endpoint_pose()
    poselist = list(tuple(pose['position']) + tuple(pose['orientation']))
    return poselist # return value is a 7x 'list'

# def calculate_tar_position(inputlist) :
# 	tar_position_in_list = 
# 	tar_position = 
# 	return tar_position

# create models
activations = dict(ReLU=ReLUActivation,
                   tanh=TanhActivation,
                   linear=LinearActivation)
activation = activations['ReLU']
model = Model(12)
for i in range(3):
    model.addLayer(DenseLayer(95,activation()))
model.addLayer(DenseLayer(7,LinearActivation()))
model.initialize(QuadraticCost())
ridge = Ridge(alpha=0.1)
# load model parameters
path1 = './model/my_model_W_{}.dat'
path2 = './model/my_model_b_{}.dat'
path3 = './model/my_model_ridge_W.dat'
path4 = './model/my_model_ridge_intercept.dat'
model_W , model_b, model_ridge_W, model_ridge_intercept = load_model(4,path1,path2,path3,path4)
# update the model
for i in range(len(model.layers)):
	model.layers[i].W = model_W[i]
	model.layers[i].b = model_b[i]
ridge.coef_ = model_ridge_W
ridge.intercept_ = model_ridge_intercept[0]

# get the data point
inputlist = ball_tracking()

# predict the configuration
tar_position_in_list = list(ridge.predict(model.predict(inputlist)))


for i in range(len(joint_names)):
    tar_position[joint_names[i]] = tar_position_in_list[i]

while True:
	cur_pos = limb.joint_angles()
	for key,value in tar_position.iteritems():
		error_new[key] = tar_position[key] - cur_pos[key]
		derror[key] = error_new[key] - error_old[key]
		error_old[key] = error_new[key]
		tar_velocity[key] = Kp*error_new[key] + Kv*derror[key]
	if not discounttime == 0:
		Kp *= discount
		discounttime -= 1

	limb.set_joint_velocities(tar_velocity)
	error_sum = np.linalg.norm(np.array(error_new.values()))
	vel_sum = np.linalg.norm(np.array(tar_velocity.values()))
	if error_sum < 0.25 and vel_sum < 0.2:
		end_time = rospy.get_time()
		break
	# rospy.sleep(1)
print('total time is ',end_time - start_time)