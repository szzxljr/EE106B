#!/usr/bin/env python
"""
Starter script for lab1. 
Author: Chris Correa
"""
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
import signal
from controllers import PDJointPositionController, PDJointVelocityController, PDJointTorqueController
#from paths import LinearPath, CircularPath, MultiplePaths
from paths import *

'''
def lookup_tag(tag_number):
    
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)
    # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
    #     print 'Frames not found'
    #     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
    #     exit(0)
    # t = rospy.Time(0)*
    # if listener.canTransform(from_frame, to_frame, t):
    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
    return (tag_pos + tag_rot)    # Return value is a 'list'
'''

if __name__ == "__main__":
    def sigint_handler(signal, frame):
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')
    time.sleep(1)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-ar_marker', '-ar', type=float, default=1)
    parser.add_argument('-controller', '-c', type=str, default='velocity') # or velocity or torque
    parser.add_argument('-arm', '-a', type=str, default='left') # or right
    args = parser.parse_args()

    arm = 'right'

    limb = baxter_interface.Limb(arm)
    kin = baxter_kinematics(arm)

    if args.controller == 'position':
        # YOUR CODE HERE
        Kp = 1
        Kv = 1
        controller = PDJointPositionController(limb, kin, Kp, Kv)
    
    if args.controller == 'velocity':
        # YOUR CODE HERE
        Kp = 1
        Kv = 1
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    if args.controller == 'torque':
        # YOUR CODE HERE
        Kp = np.diag(np.array([10 ,40,2,17,2,5,2])) #for circular path
        # Kp = np.diag(np.array([1.5,10,1,1.5,1.3,0.1,0.1]))
        Kv = np.diag(np.array([5,5,1,5,1,3,1])) # for circular path
        # Kv = np.diag(np.array([0.5,1,1,1,0.1,0.1,0.1]))
        controller = PDJointTorqueController(limb, kin, Kp, Kv)
    
    #raw_input('Press <Enter> to start')
    print('\n')
    # YOUR CODE HERE

    # cur_pos = getEndPointPosition(limb)
    # linearpath = LinearPath(kin, limb)
    linearpath_no_tracking = LinearPath_no_tracking(kin, limb)
    # circularpath = CircularPath(kin, limb)
    # multiplepaths = MultiplePaths(kin)
    
    # controller.execute_path(linearpath, None)
    controller.execute_path(linearpath_no_tracking, None)
    # controller.execute_path(circularpath, None)
    # controller.execute_path(multiplepaths, None)

    #circularPath = CircularPath(tag_pos,cur_pos)
    #controller.execute_path(circularPath, None)
    # IMPORTANT: you may find useful functions in utils.py