#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(data):
    
    pose = data.pose.pose.position
    print("X: " + str(pose.x) + "\tY: " + str(pose.y))
    quaternion = [data.pose.pose.orientation.x,
                  data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z,
                  data.pose.pose.orientation.w]
    #theta = 2 * math.acos(quaternion.w)
    print("Theta: " + str(euler_from_quaternion(quaternion)[2]))

rospy.init_node('twist_log', anonymous=True)
pose_subscriber = rospy.Subscriber('/odom', Odometry, callback)

while not rospy.is_shutdown():
    rospy.spin()
