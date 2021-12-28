#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from turtlesim.srv import Spawn
import tf2_ros
import tf2_geometry_msgs
from turtlesim.srv import TeleportAbsoluteRequest
import turtlesim.srv
import math


if __name__ == '__main__':
    rospy.init_node('turtle4_twist_remapper_node')
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', Spawn)
    spawner(5.544, 5.544, 0, 'turtle4')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    base_link_to_map_transform = TransformStamped()
    rospy.wait_for_service('turtle4/teleport_absolute')
    teleporter = rospy.ServiceProxy('turtle4/teleport_absolute', turtlesim.srv.TeleportAbsolute)

    while not rospy.is_shutdown():
        try:
            base_link_to_map_transform = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0))
        except (tf2_ros.TransformException) as e:
            print("EXCEPTION: " +  str(e))
            rate.sleep()
            continue

        pose_base_link = PoseStamped()
        pose_base_link.header.stamp = rospy.Time.now()
        pose_base_link.header.frame_id = "base_link"
        pose_base_link.pose.position.x = 0.
        pose_base_link.pose.position.y = 0.
        pose_base_link.pose.position.z = 0.
        pose_base_link.pose.orientation.x = 0.
        pose_base_link.pose.orientation.y = 0.
        pose_base_link.pose.orientation.z = 0.
        pose_base_link.pose.orientation.w = 1.
        pose_map = tf2_geometry_msgs.do_transform_pose(pose_base_link, base_link_to_map_transform)
        visualize_current_pose = TeleportAbsoluteRequest()
        visualize_current_pose.x = pose_map.pose.position.x
        visualize_current_pose.y = pose_map.pose.position.y
        quaternion = Quaternion(pose_map.pose.orientation.x, pose_map.pose.orientation.y, pose_map.pose.orientation.z, pose_map.pose.orientation.w)
        visualize_current_pose.theta = angle = 2 * math.acos(quaternion.w)
        teleporter.call(visualize_current_pose)
        rate.sleep()
