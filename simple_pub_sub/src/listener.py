#!/usr/bin/python3

import rospy
from std_msgs.msg import String

def callback(data):
    print (data)


def simple_listener():
    rospy.init_node('listener')
    rospy.Subscriber('chatter', String, callback)

    rospy.spin()

simple_listener()