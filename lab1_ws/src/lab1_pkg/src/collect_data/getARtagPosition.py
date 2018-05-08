from utils import *

signal.signal(signal.SIGINT, sigint_handler)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_node')


arm = 'right'

limb = baxter_interface.Limb(arm)
kin = baxter_kinematics(arm)

print(lookup_tag(4))