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
    def step_path(self, path, t):
        raise NotImplementedError

    def execute_path(self, path, finished, timeout=None, log=False):
        start_t = rospy.Time.now()
        times = list()
        actual_positions = list()
        actual_velocities = list()
        target_positions = list()
        target_velocities = list()
        r = rospy.Rate(200)
        while True:
            t = (rospy.Time.now() - start_t).to_sec()
            if timeout is not None and t >= timeout:
                return False
            self.step_path(path, t)  # move
            if log:
                times.append(t)
                actual_positions.append(self.current_joint_pos)
                actual_velocities.append(self.current_joint_vel)
                target_positions.append(path.target_position(t))
                target_velocities.append(path.target_velocity(t))
            if finished is not None and finished(self, path, t):
                break
            r.sleep()

        if log:
            import matplotlib.pyplot as plt

            np_actual_positions = np.zeros((len(times), 3))
            np_actual_velocities = np.zeros((len(times), 3))
            for i in range(len(times)):
                # print actual_positions[i]
                actual_positions_dict = dict((joint, actual_positions[i][j]) for j, joint in enumerate(self.limb.joint_names()))
                print "dictionary version", actual_positions_dict
                np_actual_positions[i] = self.kin.forward_position_kinematics(joint_values=actual_positions_dict)[:3]
                np_actual_velocities[i] = self.kin.jacobian(joint_values=actual_positions_dict)[:3].dot(actual_velocities[i])
            target_positions = np.array(target_positions)
            target_velocities = np.array(target_velocities)
            plt.figure()
            # print len(times), actual_positions.shape()
            plt.subplot(3,2,1)
            plt.plot(times, np_actual_positions[:,0], label='Actual')
            plt.plot(times, target_positions[:,0], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("X Position Error")

            plt.subplot(3,2,2)
            plt.plot(times, np_actual_velocities[:,0], label='Actual')
            plt.plot(times, target_velocities[:,0], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("X Velocity Error")
            
            plt.subplot(3,2,3)
            plt.plot(times, np_actual_positions[:,1], label='Actual')
            plt.plot(times, target_positions[:,1], label='Desired')
            plt.xlabel("time (t)")
            plt.ylabel("Y Position Error")

            plt.subplot(3,2,4)
            plt.plot(times, np_actual_velocities[:,1], label='Actual')
            plt.plot(times, target_velocities[:,1], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("Y Velocity Error")
            
            plt.subplot(3,2,5)
            plt.plot(times, np_actual_positions[:,2], label='Actual')
            plt.plot(times, target_positions[:,2], label='Desired')
            plt.xlabel("time (t)")
            plt.ylabel("Z Position Error")

            plt.subplot(3,2,6)
            plt.plot(times, np_actual_velocities[:,2], label='Actual')
            plt.plot(times, target_velocities[:,2], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("Z Velocity Error")

            plt.show()

        return True

class PDJointPositionController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv
        self.error_last = list(np.zeros(7))

    def step_path(self, path, t):
        # YOUR CODE HERE

        target = path.target_position(t)
        print(target[:3], target[3:])
        iks_desire = self.kin.inverse_kinematics(target[:3], target[3:])
      	#print iks_desire

        cur_pos = getEndPointPosition(self.limb)
        iks_cur = self.kin.inverse_kinematics(cur_pos[:3], cur_pos[3:])
        error = iks_desire - iks_cur
        
        derror = error - self.error_last
        self.error_last = error
        target_joint = iks_cur + self.Kp * error + self.Kv * derror
        """
        print('currentPOS:', cur_pos)
        print('targetPOS', target)
        print("ERROR:", error)
        print("DERROR:", derror)
        print('currentJoint:', iks_cur)
        print('targetjoint:', target_joint)
        print('desiretJoint:', iks_desire)
        print('\n\n')
        """
        #target_joint = iks_desire
        positions = {}
        joint = self.limb.joint_names()
        #print joint
        for i in range(len(joint)):
        	positions[joint[i]] = target_joint[i]
        #positions = {'left_w0': 0, 'left_w1': 1, 'left_w2': 0.62435204616623374, 'left_e0': 1, 'left_e1': 1, 'left_s0': 0, 'left_s1': 0}
        #print positions
        self.limb.set_joint_positions(positions)
        #self.limb.move_to_joint_positions(positions)



class PDJointVelocityController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv

    def step_path(self, path, t):
        # YOUR CODE HERE
        pass

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv

    def step_path(self, path, t):
        # YOUR CODE HERE
        """
        mass = kin.inertia()
    	J_inverse = kin.jacobian_pseudo_inverse()
    	M_tilde = (J_inverse.T.dot(mass)).dot(J_inverse)
    	"""
    	pass