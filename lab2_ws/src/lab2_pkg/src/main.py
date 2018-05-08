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
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations as tfs
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from autolab_core import RigidTransform, Point, NormalCloud, PointCloud
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from meshpy import ObjFile
warnings.filterwarnings("ignore", category=DeprecationWarning)
from visualization import Visualizer3D as vis
warnings.filterwarnings("ignore", category=DeprecationWarning)
from baxter_interface import gripper as baxter_gripper
from utils import vec, adj
import scipy
import copy
import sys
import trimesh
# import cvxpy as cvx
import Queue
from grasp_metrics import compute_force_closure, compute_gravity_resistance, compute_custom_metric

# probably don't need to change these (but confirm that they're correct)
MAX_HAND_DISTANCE = .04
MIN_HAND_DISTANCE = .01
CONTACT_MU = 0.5
CONTACT_GAMMA = 0.1

# will need to change these
OBJECT_MASS = 0.25 # kg
# approximate the friction cone as the linear combination of `NUM_FACETS` vectors
NUM_FACETS = 32
# set this to false while debugging your grasp analysis
BAXTER_CONNECTED = True
# how many to execute
NUM_GRASPS = 6
OBJECT = "pawn"

# objects are different this year so you'll have to change this
# also you can use nodes/object_pose_publisher.py instead of finding the ar tag and then computing T_ar_object in this script.
if OBJECT == "gearbox":
    MESH_FILENAME = '../objects/gearbox.obj'
    # ar tag on the paper
    TAG = 10
    # transform between the object and the AR tag on the paper
    T_ar_object = tfs.translation_matrix([-.07, -.11, 0.056])
    # how many times to subdivide the mesh
    SUBDIVIDE_STEPS = 0
elif OBJECT == 'nozzle':
    MESH_FILENAME = '../objects/nozzle.obj'
    TAG =12
    T_ar_object = tfs.translation_matrix([-.065, -.09, 0.032])
    SUBDIVIDE_STEPS = 0
elif OBJECT == "pawn":
    MESH_FILENAME = '../objects/pawn.obj'
    TAG = 14
    T_ar_object = tfs.translation_matrix([-.06, -.08, 0.091])#original [-.06, .11, 0.091]
    SUBDIVIDE_STEPS = 0

if BAXTER_CONNECTED:
    rospy.init_node('moveit_node')
    right_gripper = baxter_gripper.Gripper('right')

listener = tf.TransformListener()
from_frame = 'base'
time.sleep(1)

def rigid_transform(tag_pos, tag_rot):
    return tfs.translation_matrix(tag_pos).dot(tfs.quaternion_matrix(tag_rot))

def g_base_tag(tag_number):
    """ Returns the AR tag position in world coordinates 

    Parameters
    ----------
    tag_number : int
        AR tag number

    Returns
    -------
    :obj:`autolab_core.RigidTransform` AR tag position in world coordinates
    """
    to_frame = 'ar_marker_{}'.format(tag_number)
    if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
        print 'Frames not found'
        print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
        exit(0)
    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
    return rigid_transform(tag_pos, tag_rot)

# def lookup_tag(tag_number):
    
#     listener = tf.TransformListener()
#     from_frame = 'base'
#     to_frame = 'ar_marker_{}'.format(tag_number)
#     # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
#     #     print 'Frames not found'
#     #     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
#     #     exit(0)
#     # t = rospy.Time(0)*
#     # if listener.canTransform(from_frame, to_frame, t):
#     listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
#     t = listener.getLatestCommonTime(from_frame, to_frame)
#     tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
#     return (tag_pos + tag_rot)    # Return value is a 'list'

def close_gripper():
    """closes the gripper"""
    right_gripper.close(block=True)
    rospy.sleep(1.0)

def open_gripper():
    """opens the gripper"""
    right_gripper.open(block=True)
    rospy.sleep(1.0)

def go_to_pose(pose):
    """Uses Moveit to go to the pose specified
    Parameters
    ----------
    pose : :obj:`geometry_msgs.msg.Pose`
        The pose to move to
    """

    right_arm.set_start_state_to_current_state()
    right_arm.set_pose_target(pose)
    right_arm.plan()
    right_arm.go()

def execute_grasp(T_object_gripper):
    """takes in the desired hand position relative to the object, finds the desired hand position in world coordinates.  
       Then moves the gripper from its starting orientation to some distance behind the object, then move to the 
       hand pose in world coordinates, closes the gripper, then moves up.  
    
    Parameters
    ----------
    T_object_gripper : :obj:`autolab_core.RigidTransform`
        desired position of gripper relative to the objects coordinate frame
    """
    inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
    if inp == "exit":
        return
    # YOUR CODE HERE

def contacts_to_baxter_hand_pose(contacts, normals, approach_direction=None):
    """ takes the contacts positions in the object frame and returns the hand pose T_obj_gripper
    
    Parameters
    ----------
    contact1 : :obj:`numpy.ndarray`
        position of finger1 in object frame
    contact2 : :obj:`numpy.ndarray`
        position of finger2 in object frame
    approach_direction : :obj:`numpy.ndarray`
        there are multiple grasps that go through contact1 and contact2.  This describes which 
        orientation the hand should be in

    Returns
    -------
    :obj:`autolab_core:RigidTransform` Hand pose in the object frame
    """
    # YOUR CODE HERE
    # T_obj_gripper = ????
    target_pos = (contacts[:3] + contacts[3:]) / 2
    target_normal = - np.cross(normals[:3], normals[3:])
    target_parallel = contacts[:3] - contacts[3:]
    target_rot = np.array( [-0.603, 0.402, -0.456, 0.516]) #gearbox
    # target_rot = np.array( [-0.559, 0.829, -0.019, -0.024]) #nozzle
    # target_rot = np.array( [0.398, 0.519, 0.609, 0.449]) #pawn

    return np.append(target_pos , target_rot)

def sorted_contacts(vertices, normals, T_ar_object):
    """ takes mesh and returns pairs of contacts and the quality of grasp between the contacts, sorted by quality
    
    Parameters
    ----------
    vertices : :obj:`numpy.ndarray`
        nx3 mesh vertices
    normals : :obj:`numpy.ndarray`
        nx3 mesh normals
    T_ar_object : :obj:`autolab_core.RigidTransform`
        transform from the AR tag on the paper to the object

    Returns
    -------
    :obj:`list` of :obj:`numpy.ndarray`
        grasp_indices[i][0] and grasp_indices[i][1] are the indices of a pair of vertices.  These are randomly 
        sampled and their quality is saved in best_metric_indices
    :obj:`list` of int
        best_metric_indices is the indices of grasp_indices in order of grasp quality
    """
    
    # prune vertices that are too close to the table so you dont smack into the table
    # you may want to change this line, to be how you see fit
    possible_indices = np.r_[:len(vertices)][vertices[:,2] + T_ar_object[2,3] >= 0.03]

    # Finding grasp via vertex sampling.  make sure to not consider grasps where the 
    # vertices are too big for the gripper
    all_metrics = list()
    metric = compute_custom_metric
    grasp_indices = list()
    # for i in range(?????):
    #     candidate_indices = np.random.choice(possible_indices, 2, replace=False)
    #     grasp_indices.append(candidate_indices)
    #     contacts = vertices[candidate_indices]
    #     contact_normals = normals[candidate_indices]

    #     # YOUR CODE HERE
    #     all_metrics.append(????)

    # YOUR CODE HERE.  sort metrics and return the sorted order

    return grasp_indices, best_metric_indices


if __name__ == '__main__':
    if BAXTER_CONNECTED:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('moveit_node')
        # print('init\n\n\n\n\n')
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        right_arm.set_planner_id('RRTConnectkConfigDefault')
        right_arm.set_planning_time(5)

        # rospy.Subscriber("tf",tfMessage, callback)
        
    # Main Code
    br = tf.TransformBroadcaster()

    # SETUP
    # of = ObjFile(MESH_FILENAME)
    # mesh = of.read()
    mesh = trimesh.load(MESH_FILENAME)

    g_base_ar = g_base_tag(TAG)
    g_base_obj = g_base_ar.dot(T_ar_object)
    print('g base->ar',g_base_ar)
    print('g ar->obj',T_ar_object)
    print('g base->obj' ,g_base_obj)


    # We found this helped.  You may not.  I believe there was a problem with setting the surface normals.
    # I remember fixing that....but I didn't save that code, so you may have to redo it.  
    # You may need to fix that if you call this function.
    for i in range(SUBDIVIDE_STEPS):
        mesh = mesh.subdivide(min_tri_length=.02)

    vertices = mesh.vertices
    triangles = mesh.triangles
    # normals = mesh.normals
    normals = mesh.vertex_normals
    force_closure = compute_force_closure(vertices, normals, CONTACT_MU)
    best_contacts_objframe, best_normals_objframe = compute_custom_metric(force_closure[0], force_closure[1], CONTACT_MU)
    c1, c2 = np.append(best_contacts_objframe[:3], 1),np.append(best_contacts_objframe[3:], 1)
    best_contacts_baseframe = np.append(g_base_obj.dot(c1.reshape((4, 1)))[0:3] , g_base_obj.dot(c2.reshape((4, 1)))[0:3])
    print('best_contacts_baseframe',best_contacts_baseframe)
    best_normals_baseframe = best_normals_objframe
    hand_pos = contacts_to_baxter_hand_pose(best_contacts_baseframe, best_normals_baseframe)
    print(hand_pos)

    

    # nozzle
    # open_gripper()
    # hand_pos1 = hand_pos.copy()
    # hand_pos1[2] += .10
    # hand_pos2 = hand_pos.copy()
    # hand_pos2[2] += .1
    # hand_pos3 = hand_pos2.copy()
    # hand_pos3[1] += 0.1
    # hand_pos4 = hand_pos3.copy()
    # hand_pos4[2] -= 0.1
    # go_to_pose(list(hand_pos1))
    # rospy.sleep(1)
    # hand_pos[2] += 0.01
    # go_to_pose(list(hand_pos))
    # close_gripper()
    # go_to_pose(list(hand_pos2))
    # rospy.sleep(0.5)
    # go_to_pose(list(hand_pos3))
    # rospy.sleep(0.5)
    # go_to_pose(list(hand_pos4))
    # open_gripper()

    # # gearbox 
    # right_gripper.calibrate()
    # open_gripper()
    # hand_pos[2] += 0.063
    # hand_pos[1] -= 0.02
    # hand_pos1 = hand_pos.copy()
    # hand_pos1[0] -= .10
    # hand_pos2 = hand_pos.copy()
    # hand_pos2[2] += .1
    # hand_pos3 = hand_pos2.copy()
    # hand_pos3[1] += 0.1
    # hand_pos4 = hand_pos3.copy()
    # hand_pos4[2] -= 0.1
    # go_to_pose(list(hand_pos1))
    # rospy.sleep(1)
    # # hand_pos[2] += 0.01
    # go_to_pose(list(hand_pos))
    # close_gripper()
    # go_to_pose(list(hand_pos2))
    # rospy.sleep(0.5)
    # go_to_pose(list(hand_pos3))
    # rospy.sleep(0.5)
    # go_to_pose(list(hand_pos4))
    # open_gripper()

    # pawn
    right_gripper.calibrate()
    open_gripper()
    hand_pos[2] += 0.06
    hand_pos[1] -= 0.02
    hand_pos[0] -= 0.02
    hand_pos1 = hand_pos.copy()
    hand_pos1[0] -= .10
    hand_pos2 = hand_pos.copy()
    hand_pos2[2] += .1
    hand_pos3 = hand_pos2.copy()
    hand_pos3[1] += 0.1
    hand_pos4 = hand_pos3.copy()
    hand_pos4[2] -= 0.1
    go_to_pose(list(hand_pos1))
    rospy.sleep(1)
    # hand_pos[2] += 0.01
    go_to_pose(list(hand_pos))
    close_gripper()
    go_to_pose(list(hand_pos2))
    rospy.sleep(0.5)
    go_to_pose(list(hand_pos3))
    rospy.sleep(0.5)
    go_to_pose(list(hand_pos4))
    open_gripper()


    # ??? = sorted_contacts(???)

    # YOUR CODE HERE
    # for current_metric in ?????:
    #     # YOUR CODE HERE
        
    #     # visualize the mesh and contacts
    #     vis.figure()
    #     vis.mesh(mesh)
    #     vis.normals(NormalCloud(np.hstack((normal1.reshape(-1, 1), normal2.reshape(-1, 1))), frame='test'),
    #         PointCloud(np.hstack((contact1.reshape(-1, 1), contact2.reshape(-1, 1))), frame='test'))
    #     # vis.pose(T_obj_gripper, alpha=0.05)
    #     vis.show()
    #     if BAXTER_CONNECTED:
    #         repeat = True
    #         while repeat:
    #             execute_grasp(T_obj_gripper)
    #             repeat = bool(raw_input("repeat?"))


    # # 500, 1200
    exit()
