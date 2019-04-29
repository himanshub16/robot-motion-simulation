#!/usr/bin/env python
import math

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



def r2Callback(data):
    global listener
    listener.waitForTransform('/r2_base_link', '/r1_base_link', rospy.Time(0), rospy.Duration(5))
    trans, rot = listener.lookupTransform('/r2_base_link', '/r1_base_link', rospy.Time(0))

    dist_x, dist_y = trans[0], trans[1]

    angle_diff = math.atan2(dist_y, dist_x) # yaw diff shouldn't be considered here
    fixed_angle = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
    dist = math.hypot(dist_x, dist_y)

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
    listener = tf.TransformListener(rospy.Time(10))

    pub = rospy.Publisher('/r2/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/r2/pose', Odometry, r2Callback)
    rospy.spin()
