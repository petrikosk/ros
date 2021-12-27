#!/usr/bin/python3

#SPEED SENSOR

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import random
from tf.transformations import quaternion_from_euler

random_noise_x = rospy.get_param('/turtle_positioning_system/random_noise_x', 0,2)
random_noise_y = rospy.get_param('/turtle_positioning_system/random_noise_y', 0.2)
random_noise_theta = rospy.get_param('/turtle_positioning_system/random_noise_yaw', 0.2)

def pose_callback(msg):
    pose_to_send = PoseWithCovarianceStamped()
    quaternion = quaternion_from_euler(0.0, 0.0, msg.theta*(1 + random.gauss(0, random_noise_theta)))
    pose_to_send.header.seq = pose_to_send.header.seq + 1
    pose_to_send.header.stamp = rospy.Time.now()
    pose_to_send.header.frame_id = "map"
    pose_to_send.pose.pose.position.x = msg.x*(1 + random.gauss(0, random_noise_x))
    pose_to_send.pose.pose.position.y = msg.y*(1 + random.gauss(0, random_noise_y))
    pose_to_send.pose.pose.position.z = 0.
    pose_to_send.pose.pose.orientation.x = quaternion[0]
    pose_to_send.pose.pose.orientation.y = quaternion[1]
    pose_to_send.pose.pose.orientation.z = quaternion[2]
    pose_to_send.pose.pose.orientation.w = quaternion[3]

    pose_to_send.pose.covariance =            (random_noise_x**2, 0, 0, 0, 0, 0,
                                                0, random_noise_x**2, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, random_noise_theta**2)
    pub_pose.publish(pose_to_send)


if __name__ == '__main__':
    rospy.init_node("turtle_positioning_system")
    pub_pose = rospy.Publisher("turtle1/sensor/pose", PoseWithCovarianceStamped, queue_size=16)
    rospy.Subscriber("turtle1/pose", Pose, pose_callback)
    rospy.spin()


