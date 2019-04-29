#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

GOAL = [0, 10]


def poseCallback(data):
    global GOAL, pub
    quaternion = [data.pose.pose.orientation.x,
                  data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z,
                  data.pose.pose.orientation.w]
    euler = euler_from_quaternion(quaternion)
    x, y = data.pose.pose.position.x, data.pose.pose.position.y
    yaw = euler[2]

    angle_diff = math.atan2((GOAL[1]-y), (GOAL[0]-x)) - yaw
    fixed_angle = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
    dist = math.hypot(GOAL[1]-y, GOAL[0]-x)

    # thresholding for convergence
    speed = min(dist, 1.0) if dist > 1.0 else 0
    ang_vel = math.copysign(min(abs(fixed_angle), 1.0), fixed_angle)

    rospy.loginfo('distance : {}, vel : {}, ang vel : {}'.format(
        dist, speed, ang_vel
    ))
    vel = Twist()
    vel.linear.x = speed
    vel.angular.z = ang_vel
    pub.publish(vel)


if __name__ == '__main__':
    rospy.init_node('move_robot')
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/RosAria/pose', Odometry, poseCallback)
    rospy.spin()
