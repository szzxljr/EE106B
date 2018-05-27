import rospy
import numpy as np
from utils import *
from geometry_msgs.msg import PoseStamped
from paths import *

from baxter_pykdl import baxter_kinematics

"""
Starter script for lab1. 
Author: Chris Correa
"""
class Controller:
    # def __init__():
    #     self.current_joint_pos = [0,0,0,0,0,0,0]

    def step_path(self, path, t):
        raise NotImplementedError

    def finished(self, t_flag, t):
        if t >= t_flag:
            return True
        else:
            return False



    def execute_path(self, path, finished, timeout=None, log=True):
        start_t = rospy.Time.now()
        # print('star_t:',start_t)
        t_flag = 25
        times = list()
        actual_positions_x = list()
        actual_positions_y = list()
        # actual_velocities = list()
        target_positions_x = list()
        target_positions_y = list()
        # target_velocities = list()
        r = rospy.Rate(200)
        while True:
            t = (rospy.Time.now() - start_t).to_sec()
            # print(t_flag,t)
            if timeout is not None and t >= timeout:
                return False
            self.step_path(path, t)  # move
            if log:
                times.append(t)
                actual_positions_x.append(self.current_pos_x)
                actual_positions_y.append(self.current_pos_y)
                # actual_velocities.append(self.current_joint_vel)
                target_positions_x.append(self.target_pos_x)
                target_positions_y.append(self.target_pos_y)
                # target_velocities.append(path.target_velocity(t))
            if self.finished(t_flag, t):
                break
            r.sleep()

        if log:
            import matplotlib.pyplot as plt

            # print(target_positions_x)

            # np_actual_positions = np.zeros((len(times), 3))
            # np_actual_velocities = np.zeros((len(times), 3))
            # for i in range(len(times)):
            #     # print actual_positions[i]
            #     actual_positions_dict = dict((joint, actual_positions[i][j]) for j, joint in enumerate(self.limb.joint_names()))
            #     # print "dictionary version", actual_positions_dict
            #     np_actual_positions[i] = self.kin.forward_position_kinematics(joint_values=actual_positions_dict)[:3]
            #     np_actual_velocities[i] = self.kin.jacobian(joint_values=actual_positions_dict)[:3].dot(actual_velocities[i])
            #     print(np_actual_positions)
            # target_positions = np.array(target_positions)
            # target_velocities = np.array(target_velocities)
            plt.figure()
            # print len(times), actual_positions.shape()
            plt.subplot(2,1,1)
            # plt.plot(times, np_actual_positions[:,0], label='Actual')
            # plt.plot(times, target_positions[:,0], label='Desired')
            # plt.xlabel("Time (t)")
            # plt.ylabel("X Position Error")
            plt.plot(times, actual_positions_x, label='Actual')
            plt.plot(times, target_positions_x, label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("X Position Error")
            plt.subplot(2,1,2)
            plt.plot(times, actual_positions_y, label='Actual')
            plt.plot(times, target_positions_y, label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("X Position Error")

            # plt.subplot(3,2,2)
            # plt.plot(times, np_actual_velocities[:,0], label='Actual')
            # plt.plot(times, target_velocities[:,0], label='Desired')
            # plt.xlabel("Time (t)")
            # plt.ylabel("X Velocity Error")
            
            # plt.subplot(3,2,3)
            # plt.plot(times, np_actual_positions[:,1], label='Actual')
            # plt.plot(times, target_positions[:,1], label='Desired')
            # plt.xlabel("time (t)")
            # plt.ylabel("Y Position Error")

            # plt.subplot(3,2,4)
            # plt.plot(times, np_actual_velocities[:,1], label='Actual')
            # plt.plot(times, target_velocities[:,1], label='Desired')
            # plt.xlabel("Time (t)")
            # plt.ylabel("Y Velocity Error")
            
            # plt.subplot(3,2,5)
            # plt.plot(times, np_actual_positions[:,2], label='Actual')
            # plt.plot(times, target_positions[:,2], label='Desired')
            # plt.xlabel("time (t)")
            # plt.ylabel("Z Position Error")

            # plt.subplot(3,2,6)
            # plt.plot(times, np_actual_velocities[:,2], label='Actual')
            # plt.plot(times, target_velocities[:,2], label='Desired')
            # plt.xlabel("Time (t)")
            # plt.ylabel("Z Velocity Error")

            plt.show()

        return True

class PDJointPositionController(Controller): # PDWorkSpaceVelocityController
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv
        self.error_last = np.zeros(6)
        self.joint_names = self.limb.joint_names()
        self.current_pos_x = 0
        self.current_pos_y = 0
        self.target_pos_x = 0
        self.target_pos_y = 0
        # self.current_joint_vel = [0,0,0,0,0,0,0]

    def step_path(self, path, t):
        # YOUR CODE HERE

        target_pos = path.target_position(t) # 7x list
        cur_pos = getEndPointPosition(self.limb)  # 7x list
        self.current_pos_x = cur_pos[0]
        self.current_pos_y = cur_pos[1]
        self.target_pos_x = target_pos[0]
        self.target_pos_y = target_pos[1]
        # current_joint_pos_dic = self.limb.joint_angles()
        # for i in range(len(self.joint_names)):
        #     self.current_joint_pos[i] = current_joint_pos_dic[self.joint_names[i]]

        # joint_vel_dic = self.limb.joint_velocities()
        # for i in range(len(self.joint_names)):
        #     self.current_joint_vel[i] = joint_vel_dic[self.joint_names[i]] 

        # print(self.current_joint_vel)
        error = np.array(list(np.array(target_pos[0:3]) - np.array(cur_pos[0:3]) ) + [0,0,0] )
        derror = error - self.error_last
        self.error_last = error
        workspace_velocity = self.Kp * error + self.Kv * derror
        jacobian_inv = np.array(self.kin.jacobian_pseudo_inverse())
        target_joint_velocity = jacobian_inv.dot(workspace_velocity)
        input_joint_velocity = {}
        #print joint
        for i in range(len(self.joint_names)):
        	input_joint_velocity[self.joint_names[i]] = target_joint_velocity[i]
        self.limb.set_joint_velocities(input_joint_velocity)



class PDJointVelocityController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv
        self.error_last = np.zeros(7)
        self.joint_names = self.limb.joint_names()

        self.current_pos_x = 0
        self.current_pos_y = 0
        self.target_pos_x = 0
        self.target_pos_y = 0

    def step_path(self, path, t):
        # YOUR CODE HERE
        target_joint_angle = path.target_angle(t) # 7x list
        cur_joint_position = getEndPointPosition(self.limb) # 7x list

        self.current_pos_x = cur_joint_position[0]
        self.current_pos_y = cur_joint_position[1]
        target_position = path.target_position(t)
        self.target_pos_x = target_position[0]
        self.target_pos_y = target_position[1]

        cur_joint_angle = self.kin.inverse_kinematics(cur_joint_position[:3], path.rotation)
        error = np.array(target_joint_angle) - np.array(cur_joint_angle) # 7x np.array
        derror = error - self.error_last
        self.error_last = error
        target_joint_velocity =  self.Kp * error + self.Kv * derror
        joint_velocity = {}
        for i in range(len(self.joint_names)):
            joint_velocity[self.joint_names[i]] = target_joint_velocity[i]
        self.limb.set_joint_velocities(joint_velocity)


class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv
        self.error_last = np.zeros(7)
        self.jacobian_inverse_last = np.zeros((7,6))
        self.x_error_last = np.zeros(6)
        self.N = self.limb.joint_efforts()

        self.current_pos_x = 0
        self.current_pos_y = 0
        self.target_pos_x = 0
        self.target_pos_y = 0

    def getEndPointVelocity(self,limb):
        velocities = limb.joint_velocities()
        # velocitylist = [velocities['right_s0'],velocities['right_s1'],velocities['right_e0'],velocities['right_e1'],velocities['right_w0'],velocities['right_w1'],velocities['right_w2']]
        velocitylist = [velocities['left_s0'],velocities['left_s1'],velocities['left_e0'],velocities['left_e1'],velocities['left_w0'],velocities['left_w1'],velocities['left_w2']]

        return velocitylist # return value is a 7x 'list'
    
    def step_path(self, path, t):
        # YOUR CODE HERE
        joint_names = self.limb.joint_names()
        target_pos = path.target_position(t)

        self.target_pos_x = target_pos[0]
        self.target_pos_y = target_pos[1]

        target_pos_acc = np.array(path.target_acceleration(t))# np.array x6
        jacobian_inv = np.array(self.kin.jacobian_pseudo_inverse()) # array 7x6
        d_jacobian_inv = jacobian_inv - self.jacobian_inverse_last
        self.jacobian_inverse_last = jacobian_inv
        mass = np.array(self.kin.inertia())
        x_d_dot = path.target_velocity(t)
        cur_pos = getEndPointPosition(self.limb)  # 7x list

        self.current_pos_x = cur_pos[0]
        self.current_pos_y = cur_pos[1]

        target_pos = path.target_position(t) # 7x list
        vel_desire = self.kin.inverse_kinematics(target_pos[:3], path.rotation) #array 7
        vel_cur = self.kin.inverse_kinematics(cur_pos[:3],cur_pos[3:])
        error = vel_desire - vel_cur
        # derror = error - self.error_last
        derror = jacobian_inv.dot(x_d_dot) - self.getEndPointVelocity(self.limb)
        self.error_last = error
        ddtheta_desire = jacobian_inv.dot(target_pos_acc) + d_jacobian_inv.dot(x_d_dot)
        target_torque = mass.dot(ddtheta_desire) + np.array(self.Kp.dot(error)) + np.array(self.Kv.dot(derror))
        torque = {}
        
        # print joint_names
        for i in range(len(joint_names)):
            torque[joint_names[i]] = target_torque[i]

        # print('original_s1:',self.limb.joint_efforts()['left_e1'])
        # print('torque_s1:',torque['left_e1'])
        # print('\n')
        # print 'N', N
        # print 'target_torque', target_torque
        # print 'torque', torque
        # print '\n\n'
        self.limb.set_joint_torques(torque)

        # raw_input('enter')

        

    	