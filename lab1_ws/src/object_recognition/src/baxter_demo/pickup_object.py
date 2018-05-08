#!/usr/bin/env python
from grip_node import GripperClient

import rospy
import image_geometry
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState, DigitalIOState
from object_recognition.msg import ObjectInfo
import planning_node as pnode
import baxter_interface
import tf

camera_state_info = None
pixel_info = None
camera_info = None
button_pressed = False
global button_state
object_location = None
object_orientation = None
object_name = None
desired_object = None

# THIS WORKS
def initCamera(data):
    global camera_info
    camera_info = data

# THIS WORKS
def getCameraState(data):
    global camera_state_info
    camera_state_info = data

def getDesiredObject(data):
    global desired_object
    desired_object = data.data

# THIS WORKS
def getObjectLocation(data):
    global object_location
    global object_orientation
    global object_name
    global desired_object
    if desired_object is not None and desired_object in data.names:
        i = data.names.index(desired_object)
        object_name = data.names[i]
        x = data.x[i]
        y = data.y[i]
        object_location = [x,y]
        object_orientation = tf.transformations.quaternion_from_euler(0, 0, -data.theta[i])
        pickup()
    elif desired_object is not None and desired_object not in data.names:
        print "That object is not one of the ones on the table. Please pick again."
        desired_object = None

def pickup():
    print "------------- Object requested. -------------"
    global desired_object

    zsafe = 0.00333971663214
    zpick = -0.21

    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)
    gc = GripperClient()

    camera_x = camera_state_info.pose.position.x
    camera_y = camera_state_info.pose.position.y
    camera_z = camera_state_info.pose.position.z

    print "CAMERA X:",camera_x
    print "CAMERA Y:",camera_y
    print "CAMERA Z:",camera_z

    zoffset = -0.28 # table height in baxter's frame (see setup_notes.txt)
    pixel_size = .0025 # camera calibration (meter/pixels)
    h = camera_z-zoffset # height from table to camera
    x0 = camera_x # x camera position
    y0 = camera_y # y camera position
    x_offset = 0 # offsets
    y_offset = -.02
    height = 400 # image frame dimensions
    width = 640
    cx = object_location[0]
    cy = object_location[1]

    if object_name == 'eraser':
        cx -= 5
    # Convert pixel coordinates to baxter coordinates
    xb = (cy - (height/2))*pixel_size*h + x0 + x_offset
    yb = (cx - (width/2))*pixel_size*h + y0  + y_offset

    print "Object Location (pixels):",(cx,cy)
    print "Object Location (world):",(xb,yb)
    print "Object Orientation:",reversed(object_orientation)

    dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    dsafe_rotated = [xb, yb, zsafe, object_orientation[3], object_orientation[2], object_orientation[1], object_orientation[0]]
    dfront = [0.75, camera_y, zpick, object_orientation[3], object_orientation[2], object_orientation[1], object_orientation[0]]
    dpick = [xb, yb, zpick, object_orientation[3], object_orientation[2], object_orientation[1], object_orientation[0]]
    camera_rotated = [camera_x, camera_y, camera_z, object_orientation[3], object_orientation[2], object_orientation[1], object_orientation[0]]
    initial = [camera_x, camera_y, camera_z, 0.99, 0.01, 0.01, 0.01]

    # Publish that Baxter is about to move
    is_moving_pub.publish(True)

    print "Let's pick up the object"

    pnode.initplannode(dsafe_rotated, "left")

    gc.command(position=100.0, effort=50.0)
    gc.wait()

    pnode.initplannode(dpick, "left")

    if object_name == 'eraser':
        gc.command(position=70.0, effort=50.0)
        gc.wait()
    elif object_name == 'marker':
        gc.command(position=5.0, effort=50.0)
        gc.wait()
    elif object_name == 'screwdriver':
        gc.command(position=10.0, effort=50.0)
        gc.wait()

    pnode.initplannode(dsafe_rotated, "left")
    print "We picked up the object!"
    rospy.sleep(1)

    print "Let's put the object back down"
    pnode.initplannode(dfront, "left")

    gc.command(position=100.0, effort=50.0)
    gc.wait()

    # pnode.initplannode(dsafe_rotated, "left")
    pnode.initplannode(initial, "left")

    # Publish that Baxter has stopped moving
    is_moving_pub.publish(False)

    # Reset desired_object to None
    desired_object = None

    return

# THIS WORKS
def buttonPress(data):
    global button_state
    button_state = data.state

# THIS WORKS
def arm_setup():
    # Get desired joint values from parameter server
    left_w0 = rospy.get_param('left_w0',default =0)
    left_w1 = rospy.get_param('left_w1',default =0)
    left_w2 = rospy.get_param('left_w2',default =0)
    left_e0 = rospy.get_param('left_e0',default =0)
    left_e1 = rospy.get_param('left_e1',default =0)
    left_s0 = rospy.get_param('left_s0',default =0)
    left_s1 = rospy.get_param('left_s1',default =0)

    # Send the left arm to the desired position
    home = {'left_w0': left_w0, 'left_w1': left_w1, 'left_w2': left_w2, 'left_e0': left_e0, 'left_e1': left_e1, 'left_s0': left_s0, 'left_s1': left_s1}
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(home)

if __name__ == '__main__':
    rospy.init_node('pickup_object', log_level=rospy.INFO)

    print "Moving arm to correct location"
    arm_setup()
    rospy.Subscriber("/cameras/left_hand_camera/camera_info", CameraInfo, initCamera)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, getCameraState)
    rate = rospy.Rate(50)
    while (camera_info is None) or (camera_state_info is None):
        rate.sleep()
    rospy.Subscriber("/robot/digital_io/left_button_ok/state", DigitalIOState, buttonPress)
    rospy.Subscriber("/desired_object", String, getDesiredObject)
    rospy.Subscriber("/object_location", ObjectInfo, getObjectLocation)
    is_moving_pub = rospy.Publisher("is_moving",Bool,queue_size=10)
    is_moving_pub.publish(False)
    print "Ready to go!"
    rospy.spin()
