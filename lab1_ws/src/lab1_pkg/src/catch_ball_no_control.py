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
# limb.move_to_neutral()
cur_pos = limb.joint_angles()
# print(cur_pos)
# 	rospy.sleep(0.02)
tSpan = 0.2
start_time = rospy.get_time() #in second
tar_position = {'right_s0': 0.008436894333369775, 'right_s1': -0.5457136652902359, 'right_w0': 0.004218447166684887, 'right_w1': 1.2624661884296955, 'right_w2': -0.0038349519697135344, 'right_e0': 0.0, 'right_e1': 0.752034081260824}


# limb.set_joint_position_speed(1)
# limb.set_joint_positions(tar_position)
# limb.move_to_neutral()
for key,value in tar_position.iteritems():
	tar_velocity[key] = (tar_position[key] - cur_pos[key])/tSpan
# print(tar_velocity)
while True:
	limb.set_joint_velocities(tar_velocity)
	end_time = rospy.get_time()
	if end_time - start_time > tSpan:
		break

end_time = rospy.get_time()

print('total time is', end_time - start_time)