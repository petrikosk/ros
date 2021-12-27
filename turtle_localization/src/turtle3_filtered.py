#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from turtlesim.srv import Spawn


def twist_callback(msg):
    twist_to_turtle2 = Twist()
    twist_to_turtle2.linear.x = msg.twist.twist.linear.x
    twist_to_turtle2.angular.z = msg.twist.twist.angular.z
    pub_twist.publish(twist_to_turtle2)


if __name__ == '__main__':
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', Spawn)
    spawner(5.544, 5.544, 0, 'turtle3')

    rospy.init_node('turtle3_twist_remapper_node')
    rospy.Subscriber("odometry/filtered_twist", Odometry, twist_callback)
    pub_twist = rospy.Publisher('turtle3/cmd_vel', Twist, queue_size=1)
    rospy.spin()