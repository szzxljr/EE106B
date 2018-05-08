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
limb.move_to_neutral()