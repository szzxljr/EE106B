#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

is_moving = False

def check_moving(data):
    global is_moving
    is_moving = data.data

def poll_object_request():
    while True:
        desired_object = raw_input('Enter the object you would like to pick up (q to quit): ')
        if desired_object == 'q':
            break
        desired_object_pub.publish(desired_object)

        print "Finding and picking up ",desired_object

        rospy.sleep(2)

        while is_moving:
            pass

    print "Done!"

if __name__ == '__main__':
    rospy.init_node('request_object', log_level=rospy.INFO)

    rate = rospy.Rate(50)
    desired_object_pub = rospy.Publisher("desired_object",String,queue_size=10)

    rospy.Subscriber("is_moving",Bool,check_moving)

    poll_object_request()
