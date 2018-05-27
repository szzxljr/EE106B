import copy
import rospy
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
import numpy as np
from utils import *
from baxter_pykdl import baxter_kinematics

import math
def getEndPointPosition(limb):
    pose = limb.endpoint_pose()
    poselist = list(tuple(pose['position']) + tuple(pose['orientation']))
    return poselist # return value is a 7x 'list'


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

# while True:
cur_pos = limb.joint_angles()
# print(cur_pos)
# 	rospy.sleep(0.02)


discounttime = 2
discount = math.sqrt(0.05)
Kp = 50
Kv = 2
start_time = rospy.get_time() #in second
deltat = 0
joint_names = limb.joint_names()
# tar_position ={'right_s0': -1.0810729602622453, 'right_s1': 0.5322913333962386, 'right_w0': 0.21897575747064282, 'right_w1': 0.8666991451552588, 'right_w2': -2.508825578586594, 'right_e0': 1.055762277262136, 'right_e1': 1.3525875597179635}
tar_position = {}
position = [0.392 + 0.04*3, -0.65, 0.05-0.04*2, -0.03567816193698661, 0.6504831610794788, 0.04737272714416569, 0.7572017899052876]
# position = [0.4764627955019776, -0.65, -0.6528760306174505, -0.03567816193698661, 0.6504831610794788, 0.04737272714416569, 0.7572017899052876]
pos = position[:3]
rot = position[3:]
config = kin.inverse_kinematics(pos,rot)
print('config',config)
for i in range(len(joint_names)):
    tar_position[joint_names[i]] = config[i]

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
	t2 = time.time()
	# print('derror',derror)
	# print('\n')
	# print('tar_velocity',tar_velocity)
	# print('\n')
	# print('error_new', error_new)
	limb.set_joint_velocities(tar_velocity)
	# if (end_time - start_time > 0.4):
	# 	break
	if t2 - t0 > 5:
		break
	# print(tar_velocity)
	# print('error_new.values',np.linalg.norm(np.array(error_new.values())))
	# error_sum = np.linalg.norm(np.array(error_new.values()))
	# vel_sum = np.linalg.norm(np.array(tar_velocity.values()))
	# print(rospy.get_time() - start_time)
	# if error_sum < 0.25 and vel_sum < 0.2:
		# print("\n error_sum \n", error_sum)
		# end_time = rospy.get_time()
		# break
	# rospy.sleep(1)
# print('total time is ',end_time - start_time)
'''test if the robot can move'''
# for key,value in tar_position.iteritems():
# 	tar_velocity[key] = (tar_position[key] - cur_pos[key])/tSpan
# print(tar_velocity)
# while True:
# 	limb.set_joint_velocities(tar_velocity)
# 	if rospy.get_time() - start_time>0.2:
# 		break
