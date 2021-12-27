#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from enum import Enum
import random


class Direction(Enum):
    WEST = 3.0
    EAST = 0
    NORTH = 1.5
    SOUTH = -1.5
    NORTHWEST = 2.25
    NORTHEAST = 0.75
    SOUTHWEST = -2.25
    SOUTHEAST = -0.75


def get_turn_direction(target_angle, pose):
    # Determine shortest turn direction.
    target_deg = (3 + target_angle) * 60
    pose_deg = (3 + pose.theta) * 60

    if (target_deg - pose_deg) % 360 < 180:
        # Positive.
        return 2
    else:
        # Negative.
        return -2


def callback(data):
    global turn_direction
    global angle_target
    global positioning_ongoing
    global collision_correcting
    global collision_correcting_old

    vel_msg = Twist()
    pose = data
    
    if not collision_correcting:
        if pose.x > 10.5:
            collision_correcting = True
            if pose.y > 5:
                angle_target = Direction.SOUTHWEST.value
            else:
                angle_target = Direction.NORTHWEST.value
        elif pose.x < 0.5:
            collision_correcting = True
            if pose.y > 5:
                angle_target = Direction.SOUTHEAST.value
            else:
                angle_target = Direction.NORTHEAST.value
        elif pose.y > 10.5:
            collision_correcting = True
            if pose.x > 5:
                angle_target = Direction.SOUTHWEST.value
            else:
                angle_target = Direction.SOUTHEAST.value
        elif pose.y < 0.5:
            collision_correcting = True
            if pose.x > 5:
                angle_target = Direction.NORTHWEST.value
            else:
                angle_target = Direction.NORTHEAST.value


        else:
            # Not in collision zone - move randomly.
            vel_msg.angular.z = random.randint(-2, 2)
            vel_msg.linear.x = random.randint(0, 2)
    else:
        # Rotary positioinig of turtle.
        vel_msg.linear.x = 0 # Stop turtle forward movement.

        # Check if out of collision zone, after which continue normal random movement.
        if not positioning_ongoing and pose.x < 10 and pose.x > 1 and pose.y < 10 and pose.y > 1:
            collision_correcting = False
        #Positioning not yet started - start positioning at rising edge.
        elif collision_correcting > collision_correcting_old:
            turn_direction = get_turn_direction(angle_target, pose)
            positioning_ongoing = True
        # Position ongoing - move until position reached.
        elif positioning_ongoing:
            vel_msg.linear.x = 0
            vel_msg.angular.z = turn_direction
            # Sign check and position checking by turning direction
            if  ((pose.theta > 0 and angle_target > 0) or (pose.theta < 0 and angle_target < 0)) and \
                ((pose.theta > angle_target and turn_direction > 0) or \
                (pose.theta < angle_target and turn_direction < 0)):
                positioning_ongoing = False
        # Positioning completed, but still inside collision zone. Froward movement until out.
        else:
            vel_msg.linear.x = 2
                
        # For edge detection.
        collision_correcting_old = collision_correcting


    velocity_publisher.publish(vel_msg)

# Global var
turn_direction = 0
angle_target = 0
positioning_ongoing = False
collision_correcting = False
collision_correcting_old = False

rospy.init_node('turtlebot_auto', anonymous=True)
velocity_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, callback)

while not rospy.is_shutdown():
    rospy.spin()
        
