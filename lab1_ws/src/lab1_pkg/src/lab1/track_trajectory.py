#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import math
import sys

import rospy
import tf
import time
# from geometry_msgs.msg import Pose, PoseStamped
# import tf.transformations as tfs
# import moveit_commander
# from moveit_msgs.msg import OrientationConstraint, Constraints
# from autolab_core import RigidTransform, Point, NormalCloud, PointCloud
# import warnings
# warnings.filterwarnings("ignore", category=DeprecationWarning)
# from meshpy import ObjFile
# warnings.filterwarnings("ignore", category=DeprecationWarning)
# from visualization import Visualizer3D as vis
# warnings.filterwarnings("ignore", category=DeprecationWarning)
# from baxter_interface import gripper as baxter_gripper
# from utils import vec, adj
# import scipy
# import copy
# import sys
# import cvxpy as cvx
# import Queue
# from grasp_metrics import compute_force_closure, compute_gravity_resistance, compute_custom_metric

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D  
import numpy as np
import math

import threading
import time

listener = tf.TransformListener()
from_frame = 'base'
TAG_NUM = 2



def lookup_tag(tag_number):
    
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)

    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
    # t = listener.getLatestCommonTime(from_frame, to_frame)
    t = rospy.Time(0)
    tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
    return tag_pos, tag_rot


def lookup_tag_positon(tag_number):
    return lookup_tag(tag_number)[0]

# def lookup_tag(tag_number, listener):
    
#     listener = tf.TransformListener()
#     from_frame = 'base'
#     to_frame = 'ar_marker_{}'.format(tag_number)

#     listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
#     # t = listener.getLatestCommonTime(from_frame, to_frame)
#     t = rospy.Time(0)
#     tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
#     return tag_pos, tag_rot


# def lookup_tag_positon(tag_number, listener):
#     return lookup_tag(tag_number, listener)[0]


def track_trajectory(tag_number):
    tag_positon = lookup_tag_positon(tag_number)
    plt.close() 
    fig=plt.figure()
    ax=fig.add_subplot(111, projection = '3d')
    mes = 1
    # tag_positon = lookup_tag_positon(tag_number)
    # mesX, mexY, mexZ = tag_positon
    ax.set_xlim(-mes,mes)
    ax.set_ylim(-mes,mes)
    ax.set_zlim(-mes,mes)
    plt.grid(True) 
    plt.ion()  
    try:
        for t in range(180):
            tag_positon = lookup_tag_positon(tag_number)
            print('tag_positon',tag_positon)
            obsX, obsY, obsZ = tag_positon
            ax.scatter(obsX,obsY,obsZ,c='b')  
            plt.pause(0.001)
    except Exception as err:
        print(err)

class myThread(threading.Thread):   
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):                  
        print "Starting " + self.name
        for t in range(180):
            tag_positon = lookup_tag_positon(TAG_NUM)
            print('tag_positon',tag_positon)
        print "Exiting " + self.name
 

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    print('1')
    # track_trajectory(TAG_NUM)

    
    threads = []
    for i in range(10):
    	threads.append(myThread(i, i, i))
    for i in range(10):
    	threads[i].start()


   	# for listener in listeners:
   	# 	p = lookup_tag_positon(TAG_NUM, listener)
   	# 	print(p)





    # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
    #     print 'Frames not found'
    #     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
    #     exit(0)
    # t = rospy.Time(0)*
    # if listener.canTransform(from_frame, to_frame, t):
    # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
    # 	rospy.sleep(0.01)
    # 	lookup_tag(tag_number)



# def lookup_tag(tag_number):
#     """ Returns the AR tag position in world coordinates 

#     Parameters
#     ----------
#     tag_number : int
#         AR tag number

#     Returns
#     -------
#     :obj:`autolab_core.RigidTransform` AR tag position in world coordinates
#     """
#     to_frame = 'ar_marker_{}'.format(tag_number)
#     if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
#         print 'Frames not found'
#         print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
#         exit(0)
#     t = listener.getLatestCommonTime(from_frame, to_frame)
#     tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
#     # return rigid_transform(tag_pos, tag_rot)
#     return tag_pos, tag_rot