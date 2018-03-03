# #!/usr/bin/env python -W ignore::DeprecationWarning
# """
# Starter script for EE106B grasp planning lab
# Author: Chris Correa
# """
import numpy as np
from grasp_metrics import *
# import math
# import sys

# #import rospy
# import tf
# import time
# from geometry_msgs.msg import Pose, PoseStamped
# import tf.transformations as tfs
# import moveit_commander
# from moveit_msgs.msg import OrientationConstraint, Constraints
from autolab_core import RigidTransform, Point, NormalCloud, PointCloud
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from meshpy import ObjFile
import trimesh
warnings.filterwarnings("ignore", category=DeprecationWarning)
from visualization import Visualizer3D as vis
warnings.filterwarnings("ignore", category=DeprecationWarning)
# from baxter_interface import gripper as baxter_gripper
# from utils import vec, adj
# import scipy
# import copy
# import sys
# import cvxpy as cvx
# import Queue
# from grasp_metrics import compute_force_closure, compute_gravity_resistance, compute_custom_metric

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
BAXTER_CONNECTED = False
# how many to execute
NUM_GRASPS = 6
OBJECT = "pawn"
MESH_FILENAME = '../objects/pawn.obj'

mesh = trimesh.load(MESH_FILENAME)
# vertices = mesh.vertices
# triangles = mesh.triangles
# normals = mesh.normals
vertices = mesh.vertices
triangles = mesh.triangles
normals = -mesh.vertex_normals

of = ObjFile(MESH_FILENAME)
mesh = of.read()

# number = min(vertices.shape[0],normals.shape[0])
# vertices = vertices[:number]
# normals = normals[:number]

# print('vertices:',vertices)
# print('triangles:',triangles)
# print('normals:',normals)
print(vertices.shape)
print(normals.shape)
fc = compute_force_closure(vertices, normals, CONTACT_MU)
# print('fc:',fc)
best = compute_custom_metric(fc[0], fc[1], CONTACT_MU)
# print(best)
# print(fc[0].shape)
#print('fc', fc)

# contact1 = fc[0][100][0:3]
# contact2 = fc[0][100][3:]
# normal1 = fc[1][100][0:3]
# normal2 = fc[1][100][3:]

contact1 = best[0][0:3]
contact2 = best[0][3:]
normal1 = best[1][0:3]
normal2 =  best[1][3:]

# from autolab_core import BagOfPoints
# point = BagOfPoints(fc[0][0:3, :])

vis.figure()
vis.mesh(mesh)
vis.normals(NormalCloud(np.hstack((normal1.reshape(-1, 1), normal2.reshape(-1, 1))), frame='test'),
    PointCloud(np.hstack((contact1.reshape(-1, 1), contact2.reshape(-1, 1))), frame='test'))
# vis.pose(T_obj_gripper, alpha=0.05)
vis.show()
