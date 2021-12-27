#!/usr/bin/python3

import rospy
from std_msgs.msg import String

def talker_node():
    pub = rospy.Publisher('chatter', String, queue_size=1) # Topic is chatter
    rospy.init_node('talker') # Node name is talker
    rate = rospy.Rate(10) # attempt to publish 10 times per second
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        pub.publish(hello_str)
        rate.sleep()

talker_node()

