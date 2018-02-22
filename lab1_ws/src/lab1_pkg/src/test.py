#!/usr/bin/python
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
import numpy as np
from utils import *
from baxter_pykdl import baxter_kinematics
import signal
# from controllers import PDJointPositionController, PDJointVelocityController, PDJointTorqueController
#from paths import LinearPath, CircularPath, MultiplePaths

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

def main():
    rospy.init_node('baxter_kinematics')
    '''
    print '*** Baxter PyKDL Kinematics ***\n'
    '''
    kin = baxter_kinematics('left')
    print('\n\n')

    '''
    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()
    # FK Position
    print '\n*** Baxter Position FK ***\n'
    print kin.forward_position_kinematics()
    # FK Velocity
    # print '\n*** Baxter Velocity FK ***\n'
    # kin.forward_velocity_kinematics()
    # IK
    print '\n*** Baxter Position IK ***\n'
    pos = [0.582583, -0.180819, 0.216003]
    rot = [0.03085, 0.9945, 0.0561, 0.0829]
    print kin.inverse_kinematics(pos)  # position, don't care orientation
    print '\n*** Baxter Pose IK ***\n'
    print kin.inverse_kinematics(pos, rot)  # position & orientation
    
    # Jacobian
    print '\n*** Baxter Jacobian ***\n'
    print kin.jacobian()
    
    # Jacobian Transpose
    print '\n*** Baxter Jacobian Tranpose***\n'
    print kin.jacobian_transpose()
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    print kin.jacobian_pseudo_inverse()
    # Joint space mass matrix
    print '\n*** Baxter Joint Inertia ***\n'
    print kin.inertia()
    
    # Cartesian space mass matrix
    print '\n*** Baxter Cartesian Inertia ***\n'
    print kin.cart_inertia()
    '''
    mass = kin.inertia()
    J_inverse = kin.jacobian_pseudo_inverse()
    M_tilde = (J_inverse.T.dot(mass)).dot(J_inverse)
    #print(M_tilde)
    #print(kin.jacobian_transpose())
    #print(J_inverse)
    limb = baxter_interface.Limb('left')

    tag_pos = lookup_tag(4)
    tar_joint_angle = kin.inverse_kinematics(tag_pos[:3], tag_pos[3:])
    print(tar_joint_angle)





"""
if __name__ == "__main__":
    main()
"""