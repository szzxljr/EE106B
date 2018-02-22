#!/usr/bin/env python  
import roslib
roslib.load_manifest('lab2_pkg')
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import sys

from autolab_core import RigidTransform

class ObjectTemplate(object):
    """ Struct for specifying object templates """
    def __init__(self, name, ar_marker, R_ar_obj=np.eye(3), t_ar_obj=np.zeros(3)):
        self.name = name
        self.ar_marker = ar_marker
        self.T_ar_obj = RigidTransform(rotation=R_ar_obj, translation=t_ar_obj,
                                       from_frame=name, to_frame=ar_marker)

    @property
    def q_ar_obj(self):
        return tf.transformations.quaternion_from_matrix(self.T_ar_obj.matrix)

    @property
    def t_ar_obj(self):
        return self.T_ar_obj.translation

OBJECT_TEMPLATES = {
    ObjectTemplate(name='spray', ar_marker='ar_marker_8', t_ar_obj=[-0.089, -0.066, 0.106]),
    ObjectTemplate(name='bar_clamp', ar_marker='ar_marker_9', t_ar_obj=[-0.089, -0.074, 0.035]),
    ObjectTemplate(name='mount2', ar_marker='ar_marker_10', t_ar_obj=[-0.103, -0.064, 0.038])
}

if __name__ == '__main__':
    rospy.init_node('object_pose_publisher')

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
 
    print 'Publishing object pose'
    
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            for object_template in OBJECT_TEMPLATES:
                broadcaster.sendTransform(object_template.t_ar_obj,object_template. q_ar_obj, listener.getLatestCommonTime('base', 'left_hand_camera'), object_template.name, object_template.ar_marker)
        except:
            continue
        rate.sleep()