#!/usr/bin/python3

#SPEED SENSOR

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped
import random

random_noise_linear = 0.05
random_noise_angular = 0.02

def pose_callback(msg):
    twist_to_send = TwistWithCovarianceStamped()
    twist_to_send.header.seq = twist_to_send.header.seq + 1
    twist_to_send.header.stamp = rospy.Time.now()
    twist_to_send.header.frame_id = "base_link"
    twist_to_send.twist.twist.linear.x = msg.linear_velocity*(1 + random.gauss(0, random_noise_linear))
    twist_to_send.twist.twist.angular.z = msg.angular_velocity*(1 + random.gauss(0, random_noise_angular))
    twist_to_send.twist.covariance =            (random_noise_linear**2, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                random_noise_angular**2, 0, 0, 0, 0, 0)
    
    pub_twist.publish(twist_to_send)


if __name__ == '__main__':
    rospy.init_node("turtle_odometry")
    pub_twist = rospy.Publisher("turtle1/sensor/twist", TwistWithCovarianceStamped, queue_size=16)
    rospy.Subscriber("turtle1/pose", Pose, pose_callback)
    rospy.spin()


