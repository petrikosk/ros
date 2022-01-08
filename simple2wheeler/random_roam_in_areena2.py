#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from enum import Enum
import random


class Direction(Enum):
    WEST = 0
    EAST = 3
    NORTH = -1.5
    SOUTH = 1.5
    NORTHWEST = -0.75
    NORTHEAST = -2.25
    SOUTHWEST = 0.75
    SOUTHEAST = 2.25


def get_turn_direction(target_angle, theta):
    # Determine shortest turn direction.
    target_deg = (3 + target_angle) * 60
    pose_deg = (3 + theta) * 60

    if (target_deg - pose_deg) % 360 < 180:
        # Positive.
        return 1
    else:
        # Negative.
        return -1


def callback(data):
    global turn_direction
    global angle_target
    global positioning_ongoing
    global collision_correcting
    global collision_correcting_old
    global pit

    vel_msg = Twist()
    
    pose = data.pose.pose.position
    quaternion = [data.pose.pose.orientation.x,
                  data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z,
                  data.pose.pose.orientation.w]
    theta = euler_from_quaternion(quaternion)[2]
    
    if not collision_correcting:
        if pose.x > 2.4:
            collision_correcting = True
            if pose.y > 1.5:
                angle_target = Direction.SOUTHWEST.value
            else:
                angle_target = Direction.NORTHWEST.value
        elif pose.x < 0.7:
            collision_correcting = True
            if pose.y > 1.5:
                angle_target = Direction.SOUTHEAST.value
            else:
                angle_target = Direction.NORTHEAST.value
        elif pose.y > 2.4:
            collision_correcting = True
            if pose.x > 1.5:
                angle_target = Direction.SOUTHWEST.value
            else:
                angle_target = Direction.SOUTHEAST.value
        elif pose.y < 0.7:
            collision_correcting = True
            if pose.x > 1.5:
                angle_target = Direction.NORTHWEST.value
            else:
                angle_target = Direction.NORTHEAST.value
            # Avoid pit in lower left corner
        elif pose.y < 1.3 and pose.x < 1.3:
            collision_correcting = True
            pit = True
            angle_target = Direction.NORTHEAST.value


        else:
            # Not in collision zone - move randomly.
            vel_msg.angular.z = random.randint(-1, 1)
            vel_msg.linear.x = float(random.randint(1, 5)) / 10
    else:
        # Rotary positioinig of robot.
        vel_msg.linear.x = 0 # Stop forward movement.

        # Check if out of collision zone, after which continue normal random movement.
        if not positioning_ongoing and ((pose.x < 2.399 and pose.x > 0.701 and pose.y < 2.399 and pose.y > 0.701 and not pit) or \
            (pose.y > 1.301 and pose.x > 1.301 and pit)):
            collision_correcting = False
            pit = False
        # Positioning not yet started - start positioning at rising edge.
        elif collision_correcting > collision_correcting_old:
            turn_direction = get_turn_direction(angle_target, theta)
            positioning_ongoing = True
        # Position ongoing - move until position reached.
        elif positioning_ongoing:
            vel_msg.linear.x = 0
            vel_msg.angular.z = turn_direction
            # Sign check and position checking by turning direction
            if  ((theta > 0 and angle_target > 0) or (theta < 0 and angle_target < 0)) and \
                ((theta > angle_target and turn_direction > 0) or \
                (theta < angle_target and turn_direction < 0)):
                positioning_ongoing = False
        # Positioning completed, but still inside collision zone. Froward movement until out.
        else:
            vel_msg.linear.x = 0.2
                
        # For edge detection.
        collision_correcting_old = collision_correcting


    velocity_publisher.publish(vel_msg)

# Global var
turn_direction = 0
angle_target = 0
positioning_ongoing = False
collision_correcting = False
collision_correcting_old = False
pit = False

rospy.init_node('auto_roam', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('/odom', Odometry, callback)

while not rospy.is_shutdown():
    rospy.spin()
        
