#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Quaternion
from turtlesim.srv import Spawn, TeleportAbsolute, TeleportAbsoluteRequest
import math

def pose_callback(msg):
        
    visualize_current_pose = TeleportAbsoluteRequest()
    visualize_current_pose.x = msg.pose.pose.position.x
    visualize_current_pose.y = msg.pose.pose.position.y
    quaternion = Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    visualize_current_pose.theta = angle = 2 * math.acos(quaternion.w)
    teleporter.call(visualize_current_pose)


if __name__ == "__main__":
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', Spawn)
    spawner(5.544, 5.544, 0, 'turtle3')
    rospy.wait_for_service('turtle3/teleport_absolute')
    teleporter = rospy.ServiceProxy('turtle3/teleport_absolute', TeleportAbsolute)
    rospy.init_node('turtle3_pose_remapper_node')
    rospy.Subscriber("turtle1/sensor/pose", PoseWithCovarianceStamped, pose_callback)
    rospy.spin()


    