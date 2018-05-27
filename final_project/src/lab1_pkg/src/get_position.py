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
from paths import *
import math


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
print('angle_pos',cur_pos)
print('*******************')
cur_joint_position = getEndPointPosition(limb)
print('position',cur_joint_position)
print('*******************')
pos = cur_joint_position[:3]
rot = cur_joint_position[3:]
inverse = kin.inverse_kinematics(pos,rot)
print('inverse_kinematics',inverse)
for i in range(len(joint_names)):
    tar_position[joint_names[i]] = config[i]

