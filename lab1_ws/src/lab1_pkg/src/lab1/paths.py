import rospy
import numpy as np
import math
from utils import *
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

# import IPython
import tf
import tf2_ros
import time
from baxter_pykdl import baxter_kinematics
import signal



# from controllers import PDJointPositionController, PDJointVelocityController, PDJointTorqueController
#from paths import LinearPath, CircularPath, MultiplePaths

"""
Starter script for lab1. 
Author: Chris Correa
"""

# IMPORTANT: the init methods in this file may require extra parameters not 
# included in the starter code.  
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

def getEndPointPosition(limb):
    pose = limb.endpoint_pose()
    poselist = list(tuple(pose['position']) + tuple(pose['orientation']))
    return poselist # return value is a 7x 'list'


class MotionPath:
    def target_position(self, time):
        raise NotImplementedError

    def target_velocity(self, time):
        raise NotImplementedError

    def target_acceleration(self, time):
        raise NotImplementedError

class LinearPath(MotionPath):
    def __init__(self, kin, limb):
        self.kin = kin
        self.limb = limb
        self.end_point = getEndPointPosition(self.limb)
        # self.rotation = getEndPointPosition(self.limb)[3:]
        self.rotation = self.end_point[3:]
        self.cur_pos = self.end_point[:3]
        self.target_pos = [0,0,0,0,0,0,0]
        self.target_pos_torque = [0,0,0,0,0,0,0]
        #self.target[2] += 0.2
        #self.init_pos = cur_pos
        #self.velocity = list((np.array(tag_pos[0:3]) - np.array(cur_pos[0:3])) / dfactor)

    def target_position(self, t):
        tag_pos = lookup_tag(7)
        tag_pos[2] = tag_pos[2] + 0.15

        self.target_pos = tag_pos

        # return list(np.array(self.init_pos[0:3]) + np.array(self.velocity) * 2) + self.init_pos[3:]
        return tag_pos # 7 list

    def target_velocity(self, t):
        velocity_vec = -(np.array(self.cur_pos) - np.array(self.target_pos[:3]))
        x_d_dot = list(velocity_vec/np.linalg.norm(velocity_vec))+[0,0,0]
        return np.array(x_d_dot)

    def target_angle(self, t):
        tag_pos = lookup_tag(7)
        tag_pos[2] = tag_pos[2] + 0.15
        # good
        position = tag_pos[0:3]
        counter = 0
        # print tag_pos
        while True:
            counter += 1
            if counter > 200:
                break
            tar_joint_angle = self.kin.inverse_kinematics(position, self.rotation)
            if tar_joint_angle is not None:
                break
        # print(tar_joint_angle)
        return tar_joint_angle # return value is a list contains 7 joint angles

    def target_acceleration(self, t):
        return np.zeros(6)

class LinearPath_no_tracking(MotionPath):
    def __init__(self, kin, limb):
        self.kin = kin
        self.limb = limb
        self.end_point = getEndPointPosition(self.limb)
        # self.rotation = getEndPointPosition(self.limb)[3:]
        self.rotation = self.end_point[3:]
        self.cur_pos = self.end_point[:3]
        # self.target_pos = lookup_tag(4)
        self.target_pos = [0.988, -0.426, 0.359,-0.599, 0.680, -0.377, 0.193]
        self.target_pos[2] += 0.15
        self.target_pos_torque = [0,0,0,0,0,0,0]

    def target_position(self, t):
        # tag_pos = lookup_tag(4)
        # tag_pos[2] = tag_pos[2] + 0.15
        # self.target_pos = tag_pos

        # return list(np.array(self.init_pos[0:3]) + np.array(self.velocity) * 2) + self.init_pos[3:]
        return self.target_pos # 7 list
    
    def target_velocity(self, t):
        self.end_point = getEndPointPosition(self.limb)
        self.cur_pos = self.end_point[:3]
        velocity_vec = -(np.array(self.cur_pos) - np.array(self.target_pos[:3]))
        x_d_dot = list(velocity_vec/np.linalg.norm(velocity_vec))+[0,0,0]
        return np.array(x_d_dot)

    def target_angle(self, t):
        position = self.target_pos[0:3]
        counter = 0
        # print tag_pos
        while True:
            counter += 1
            if counter > 200:
                break
            tar_joint_angle = self.kin.inverse_kinematics(position, self.rotation)
            if tar_joint_angle is not None:
                break
        # print(tar_joint_angle)
        return tar_joint_angle # return value is a list contains 7 joint angles

    def target_acceleration(self, t):
        return np.zeros(6)
    

class CircularPath(MotionPath):
    def __init__(self, kin, limb, radius=0.15):
        self.radius = radius
        self.kin = kin
        self.theta = 0
        self.angular_v = 0.5
        self.limb = limb
        self.center_pos = lookup_tag(7)
        self.center_pos[2] = self.center_pos[2]+0.2
        self.target_pos = self.center_pos
        self.rotation = getEndPointPosition(self.limb)[3:]
                # if radius is not None:
        #     self.circle_pos = self.set_circle_pos(radius)
        # else:
        #     pass
    # def set_circle_pos(self, radius):
    #     tot_dis = sqrt(self.center_pos^2 + self.init_pos^2)
    #     ratio = 1 - radius / tot_dis
    #     return self.init_pos + (self.center_pos - self.init_pos) * ratio 

    # def move_to_circle(self):
    #     clinearpath = LinearPath(self.center_pos, self.circle_pos)

    def target_position(self, t):
        self.theta = np.remainder(self.angular_v*t,2*np.pi)
        x = self.radius*np.cos(self.theta) + self.center_pos[0]
        y = self.radius*np.sin(self.theta) + self.center_pos[1]
        target_pos = [x,y] + [self.center_pos[2]] + self.rotation

        # self.center_pos = center_pos
        self.target_pos = target_pos
        return target_pos # 7x list

    def target_angle(self, t):
        tag_pos = self.target_position(t)
        # good
        position = tag_pos[0:3]
        rotation = tag_pos[3:]
        tar_joint_angle = self.kin.inverse_kinematics(position, rotation)
        # print(tar_joint_angle)
        return tar_joint_angle # return value is a list contains 7 joint angles

    def target_velocity(self, t):
        w = np.array([0,0,self.angular_v])
        r = np.array(self.center_pos[0:3]) - np.array(self.target_pos[0:3])
        x_d_dot = list(hat(w).dot(r))+[0,0,0]
        return np.array(x_d_dot)


    def target_acceleration(self, t):
        acc_vec = (np.array(self.center_pos)[:3] - np.array(self.target_pos)[:3])
        acc_dir = acc_vec / np.linalg.norm(acc_vec)
        acc = self.angular_v * self.angular_v * self.radius * acc_dir
        return np.array(list(acc) + [0, 0, 0])

# You can implement multiple paths a couple ways.  The way I chose when I took
# the class was to create several different paths and pass those into the 
# MultiplePaths object, which would determine when to go onto the next path.

class MultiplePaths(MotionPath):
    def __init__(self, kin):
        self.kin = kin
        self.tag_poses = []
        for i in range(4):
            tag_pos = lookup_tag(i)
            tag_pos[2] += 0.2
            self.tag_poses.append(tag_pos)
        self.index = 0
        self.limb = baxter_interface.Limb('left')
        """
    def target_position(self, t, dfactor=50):
        # print('t:',t)
        print('t//1',t//1)
        if np.remainder(t//1, dfactor) < 1:
            self.index = np.remainder(self.index + 1, 4)
        print('index:',self.index)
        return self.tag_poses[int(self.index)]
        """
    def target_position(self, t):
        cur_pos = getEndPointPosition(self.limb)
        tar_pos = self.tag_poses[self.index]
        error = 0.01
        if np.linalg.norm(np.array(cur_pos[:2]) - np.array(tar_pos[:2])) < error:
            self.index = np.remainder(self.index + 1, 4)
        # print tar_pos
        return self.tag_poses[self.index]
    
    # def target_velocity(self, t):
    #     # return lookup_tag(4)
    #     return list(np.array(self.init_pos[0:3]) + np.array(self.velocity) * 2) + self.init_pos[3:]
    def target_angle(self, t):
        if np.remainder(t, dfactor) < 0.1:
            self.index = np.remainder(self.index + 1, 4)
            tar_pos = self.tag_poses[self.index]
        position = tag_pos[0:3]
        rotation = tag_pos[3:]
        tar_joint_angle = self.kin.inverse_kinematics(position, rotation)
        print(tar_joint_angle)
        return tar_joint_angle # return value is a list contains 7 joint angles

    def target_acceleration(self, t):
        return 0