import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import baxter_interface
from baxter_pykdl import baxter_kinematics

def getEndPointPosition(limb):
    pose = limb.endpoint_pose()
    poselist = list(tuple(pose['position']) + tuple(pose['orientation']))
    return poselist # return value is a 7x 'list'
    
rospy.init_node('moveit_node')
arm = 'right'
limb = baxter_interface.Limb(arm)
kin = baxter_kinematics(arm)
cur_pos = limb.joint_angles()
cur_joint_position = getEndPointPosition(limb)
pos = cur_joint_position[:3]
rot = cur_joint_position[3:]
# pos = [0.9756087202365216, -0.49638535803286743, -0.15694567939226278]
# rot = [0.01973686805892735, 0.7033605384822768, -0.0012924354324438771, 0.710558047281008]
start = rospy.get_time()
print(rot)
config = kin.inverse_kinematics(pos,rot)
print('***************************************')
print('inverse_kinematics result', config)
print('***************************************')
print('limb.joint_angles',cur_pos)
print('***************************************')
end = rospy.get_time()
print('total time is ', end - start)