import numpy as np
import math
from utils import *

"""
Starter script for lab1. 
Author: Chris Correa
"""

# IMPORTANT: the init methods in this file may require extra parameters not 
# included in the starter code.  

def getEndPointPosition(limb):
    pose = limb.endpoint_pose()
    poselist = list(tuple(pose['position']) + tuple(pose['orientation']))
    return poselist

class MotionPath:
    def target_position(self, time):
        raise NotImplementedError

    def target_velocity(self, time):
        raise NotImplementedError

    def target_acceleration(self, time):
        raise NotImplementedError

class LinearPath(MotionPath):
    def __init__(self, tag_pos, cur_pos, dfactor=5):
        self.target = tag_pos 
        self.target[2] += 0.2
        self.init_pos = cur_pos
        self.velocity = list((np.array(tag_pos[0:3]) - np.array(cur_pos[0:3])) / dfactor)

    def target_position(self, t):
        #return self.target
        return list(np.array(self.init_pos[0:3]) + np.array(self.velocity) * 5) + self.init_pos[3:]
        # return self.target
    '''
    def target_velocity(self, t):
        pass

    def target_acceleration(self, t):
        pass
    '''
"""
class CircularPath(MotionPath):
	def __init__():

    def target_position(self, t):

    def target_velocity(self, t):

    def target_acceleration(self, t):

        

# You can implement multiple paths a couple ways.  The way I chose when I took
# the class was to create several different paths and pass those into the 
# MultiplePaths object, which would determine when to go onto the next path.

class MultiplePaths(MotionPath):
	def __init__(self, paths):

	def get_current_path(self, time):
"""