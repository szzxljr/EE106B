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
rospy.init_node('moveit_node')
arm = 'right'
limb = baxter_interface.Limb(arm)
kin = baxter_kinematics(arm)

# num = int(sio.loadmat('./inputValue.mat').keys()[-4])
if not os.stat('./new_data/inputValue.mat').st_size == 0:
	old_inputValue = sio.loadmat('./new_data/inputValue.mat')['inputValue']
	old_angel_pos =  sio.loadmat('./new_data/angle_pos.mat')['angle_pos']
	old_threeD_pos = sio.loadmat('./new_data/threeD_pos.mat')['threeD_pos']
	# print('old_inputValue',old_inputValue)
	old_inputValue = list(np.array(old_inputValue).reshape(1,-1))
	old_angel_pos = list(np.array(old_angel_pos).reshape(1,-1))
	old_threeD_pos = list(np.array(old_threeD_pos).reshape(1,-1))

	old_inputValue = list(np.array(old_inputValue).flatten())
	old_angel_pos = list(np.array(old_angel_pos).flatten())
	old_threeD_pos = list(np.array(old_threeD_pos).flatten())


else:
	old_inputValue, old_angel_pos, old_threeD_pos = [],[],[]
	num = 1


while True:

	inputvalue = ball_tracking()
	print('**************************************')
	print('press o if this data is right')
	print('**************************************')
	key = raw_input()
	if not key == 'o':
		print('Try again')

	else:
		# num += 1
		# r = rospy.Rate(1000)
		joint_names = limb.joint_names()
		old_inputValue.extend(inputvalue)
		old_angel_pos.extend(limb.joint_angles().values())
		old_threeD_pos.extend(getEndPointPosition(limb))
		sio.savemat('./new_data/inputValue.mat',{'inputValue':list(np.array(old_inputValue).reshape(-1,10))})
		sio.savemat('./new_data/angle_pos.mat',{'angle_pos':list(np.array(old_angel_pos).reshape(-1,7))})
		sio.savemat('./new_data/threeD_pos.mat',{'threeD_pos':list(np.array(old_threeD_pos).reshape(-1,7))})
		# print('old_angel_pos', old_angel_pos)
		# print('old_threeD_pos', old_threeD_pos)

	print('**************************************')
	print('press p to stop and any other key to open the camera')
	print('**************************************')
	key2 = raw_input()
	if key2 == 'p':
		# print('old_inputValue', old_inputValue)
		# sio.savemat('./inputValue.mat',{'inputValue':list(np.array(old_inputValue).reshape(-1,12))})
		# sio.savemat('./angle_pos.mat',{'angle_pos':list(np.array(old_angel_pos).reshape(-1,7))})
		# sio.savemat('./threeD_pos.mat',{'threeD_pos':list(np.array(old_threeD_pos).reshape(-1,7))})
		break