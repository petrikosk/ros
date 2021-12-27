#!/usr/bin/python3

#POSITION SENSOR

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
import tf_conversions
import tf2_ros
import random

random_noise_linear = 0.2
random_noise_angular = 0.2
random_noise_x = float(rospy.get_param('/turtle1_odometry2_node/random_noise_x', '0.2'))
random_noise_y = float(rospy.get_param('/turtle1_odometry2_node/random_noise_y', '0.2'))
random_noise_yaw = float(rospy.get_param('/turtle1_odometry2_node/random_noise_yaw', '0.2'))

def pose_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'turtleTarget'

    t.child_frame_id = "turtleSrc"
    
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node("turtle_odometry2")
    turtle_vel  = rospy.Publisher("turtle1/sensor2/twist", TwistWithCovarianceStamped, queue_size=1)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('turtleSrc', 'turtleTarget', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("EXCEPTION: " +  str(e))
            rate.sleep()
            continue

        twist_to_send = TwistWithCovarianceStamped()
        twist_to_send.header.seq = twist_to_send.header.seq + 1
        twist_to_send.header.stamp = rospy.Time.now()
        twist_to_send.header.frame_id = "base_link"
        twist_to_send.twist.twist.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        twist_to_send.twist.twist.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        twist_to_send.twist.covariance = (          0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0
                                                    )
        turtle_vel.publish(twist_to_send)

        rate.sleep()

