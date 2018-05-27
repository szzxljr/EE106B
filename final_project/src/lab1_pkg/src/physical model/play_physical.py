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
import time
# from backprop import *
# import backprop
from sklearn.linear_model import Ridge

import matplotlib.pyplot as plt
moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('moveit_node')
# print('init\n\n\n\n\n')


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
# cur_pos = getEndPointPosition(limb)
discounttime = 2
discount = math.sqrt(0.05)
Kp = 50
Kv = 2
start_time = rospy.get_time() #in second
deltat = 0
joint_names = limb.joint_names()
# tar_position ={'right_s0': 0.002684466378799474, 'right_s1': -0.5518495884417776, 'right_w0': -0.006135923151541655, 'right_w1': 1.254412789293297, 'right_w2': 0.005368932757598948, 'right_e0': -0.0019174759848567672, 'right_e1': 0.7531845668517382}
tar_position = {}
g = 9.81
error_last = np.zeros(6)
input_joint_velocity = {}
camera_to_base = [0.803, -1.432, -0.194, -0.601, 0.003, 0.040, 0.799]
R_camera_to_base = np.array([get_R_from_quaternion(camera_to_base[3:])])
p_camera_to_base = np.array([camera_to_base[0:3]])

print(p_camera_to_base)
g_base_camera = getg(R_camera_to_base, p_camera_to_base)

def get_real_pos_from_px_in_camera_x(x):
	x_real_in_camera = (x - 320)*0.14/(320-207)
	return x_real_in_camera

def get_real_pos_from_px_in_camera_y(y):
	y_real_in_camera = (y - 240)*(0.15 - 0.031)/90
	return y_real_in_camera


def get_position_in_space(x,y,z,g = g_base_camera):
	position = list(g_base_camera*np.array([[x],[y],[z],1]))[0:3]
	return position



def ball_tracking():
	# ap = argparse.ArgumentParser()
	# ap.add_argument("-v", "--video", help="path to the (optional) video file")
	# args = vars(ap.parse_args())
	vx,vy,vz = 0.0,0.0,0.0
	greenLower = (25, 51, 30)
	greenUpper = (100, 255, 178)
	x_list = []
	y_list = []
	r_list = []
	time_list = []
	pos_array = np.zeros((4,4))
	pos_array[3,:] = 1
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
	            t = time.time()
	            # print('x:',x)
	            # print('y',y)
	            print('radius',radius)
	            x_list.append(x)
	            y_list.append(y)
	            r_list.append(radius)
	            time_list.append(t)

	    cv2.imshow("Frame", frame)
	    # cv2.imshow("Mask", mask)

	    # key = cv2.waitKey(1) & 0xFF
	    if len(x_list) == 6:
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
	# x_list.pop(0)
	# y_list.pop(0)
	# r_list.pop(0)
	# time_list.pop(0)
	time_list.pop(0)
	time_list.pop(0)
	for i in range(len(x_list)):
	    cv2.circle(frame,(int(x_list[i]),int(y_list[i])),int(r_list[i]),(0,0,255),1)
	cv2.imshow("Frame", frame)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	delta_t_list = [time_list[1]-time_list[0],time_list[2]-time_list[1],time_list[3]-time_list[2]]
	x_real_in_camera = np.array([map(get_real_pos_from_px_in_camera_x, x_list)])
	y_real_in_camera = np.array([map(get_real_pos_from_px_in_camera_y, y_list)])
	z_real_in_camera = 0.81
	pos_array[0,:] = x_real_in_camera
	pos_array[1,:] = y_real_in_camera
	pos_array[2,:] = 0.81
	pos_array[3,:] = 1.0
	

	# print('pos_array',pos_array)
	# print('delta_t_list',delta_t_list)
	# pos_in_space = np.dot(g_base_camera,pos_array)
	vy = ((pos_array[1,1]-pos_array[1,0])/delta_t_list[0] + (pos_array[1,2]-pos_array[1,1])/delta_t_list[1])/2 - g*delta_t_list[0]
	for n in range(3):	
		vx += (1.0/3)*(pos_array[0,n+1]-pos_array[0,n])/delta_t_list[n]
		# vz += (1.0/3)*(pos_array[2,n+1]-pos_array[2,n])/delta_t_list[n]
	total_t = -2*vy/g
	print('vy',vy)
	print('total_t',total_t)
	add_time = total_t + total_t/10.0
	predict_x_in_camera = pos_array[0,0] + vx*add_time
	predict_y_in_camera = pos_array[1,0] + vy*add_time + 1.0/2*g*add_time*add_time
	predict_z_in_camera = pos_array[2,0]
	predict_array_in_camera = np.array([[predict_x_in_camera],[predict_y_in_camera],[predict_z_in_camera],[1]])
	predict_array = np.dot(g_base_camera,predict_array_in_camera)
	# print(pos_array_in_camera)
	# print(g_base_camera)
	# print(predict_array)
	# print(delta_t_list)
	pos = [predict_array[0,0],predict_array[1,0],predict_array[2,0]]
	rot = [-0.08327610070874578, 0.7039092189481112, 0.027853081276324902, 0.7048411937408463]
	# print('predict_x_in_camera', predict_x_in_camera)
	# print('predict_y_in_camera', predict_y_in_camera)
	# plt.plot(predict_x_in_camera, predict_y_in_camera, 'ro', pos_array[0,:], pos_array[1,:], 'bo')
	# plt.show()
	return pos, rot

# print(ball_tracking())

# ball_pos_in_camera = [x_real_in_camera,y_real_in_camera,z_real_in_camera,0,0,0,1]
# R_ball_pos_in_camera = np.eye(3)
# p_ball_pos_in_camera = np.transpose(np.array(ball_pos_in_camera[0:3]))
# ball_to_base = g_base_camera*np.array([[x_real_in_camera],[y_real_in_camera],[z_real_in_camera],[1]])
tar_pos, rot = ball_tracking()

'''
move part
'''
# print('pos',pos)
# print('rot',rot)
print('\n\n')

# ik = []
# while ik is None or ik[]:
# 	ik = kin.inverse_kinematics(pos,rot)

configs =[]
config = []
i = 3
while i:
	con = kin.inverse_kinematics(tar_pos,rot)
	if con is not None:
		configs += list(con)
		i -= 1
for i in range(7):
	lst = [configs[i], configs[i + 7], configs[i + 14]]
	lst.sort()
	config.append(lst[1])
config = np.array(config)
# while not config:
# 	config = kin.inverse_kinematics(tar_pos,rot)


# print('config', config)
# config1 = kin.inverse_kinematics(pos,rot)
for i in range(len(joint_names)):
    tar_position[joint_names[i]] = config[i]
# print('config',config)
# print(joint_names)

'''
***************************position controller*********************
t0 = time.time()
while True:
    cur_pos = getEndPointPosition(limb)
    error = np.array(list(np.array(tar_pos[0:3]) - np.array(cur_pos[0:3])) + [0,0,0] )
    derror = error - error_last
    error_last = error
    workspace_velocity = Kp * error + Kv * derror
    jacobian_inv = np.array(kin.jacobian_pseudo_inverse())
    target_joint_velocity = jacobian_inv.dot(workspace_velocity)

    #print joint
    for i in range(len(joint_names)):
    	input_joint_velocity[joint_names[i]] = target_joint_velocity[i]
    limb.set_joint_velocities(input_joint_velocity)
    if not discounttime == 0:
		Kp *= discount
		discounttime -= 1
    t2 = time.time()
    if t2 - t0 > 7:
        break
'''


# ******************angle controller**********************    
print('tar_position',tar_position)
print('\n')
print('cur_pos',cur_pos)
print('\n\n')
for key,value in tar_position.iteritems():
		error_old[key] = (tar_position[key] - cur_pos[key])

t0 = time.time()
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
	# print(rospy.get_time() - start_time)
	t2 = time.time()
	if t2 - t0 > 5:
		break


# import sys
# import moveit_commander
# from moveit_msgs.msg import OrientationConstraint, Constraints
# def go_to_pose(pose):
#     """Uses Moveit to go to the pose specified
#     Parameters
#     ----------
#     pose : :obj:`geometry_msgs.msg.Pose`
#         The pose to move to
#     """

#     right_arm.set_start_state_to_current_state()
#     right_arm.set_pose_target(pose)
#     right_arm.plan()
#     right_arm.go()

# moveit_commander.roscpp_initialize(sys.argv)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# right_arm = moveit_commander.MoveGroupCommander('right_arm')
# right_arm.set_planner_id('RRTConnectkConfigDefault')
# right_arm.set_planning_time(5)
# go_to_pose(tar_pos + rot)