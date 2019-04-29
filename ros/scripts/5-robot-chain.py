#!/usr/bin/env python
import math

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pubs = {}
GOAL_MAP = {
    'r2': 'r1',
    'r3': 'r2',
    'r4': 'r3',
    'r5': 'r4',
}

def poseCallbackWrapper(robot_name):
    def callback(data):
        global listener
        listener.waitForTransform('/{}_base_link'.format(robot_name),
                                  '/{}_base_link'.format(GOAL_MAP[robot_name]),
                                  rospy.Time(0), rospy.Duration(5))
        trans, rot = listener.lookupTransform('/{}_base_link'.format(robot_name),
                                              '/{}_base_link'.format(GOAL_MAP[robot_name]),
                                              rospy.Time(0))

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
        pubs[robot_name].publish(vel)

    return callback

if __name__ == '__main__':
    rospy.init_node('move_robot_chain')
    listener = tf.TransformListener(rospy.Time(10))

    pubs['r2'] = rospy.Publisher('/r2/cmd_vel', Twist, queue_size=10)
    pubs['r3'] = rospy.Publisher('/r3/cmd_vel', Twist, queue_size=10)
    pubs['r4'] = rospy.Publisher('/r4/cmd_vel', Twist, queue_size=10)
    pubs['r5'] = rospy.Publisher('/r5/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/r2/pose', Odometry, poseCallbackWrapper('r2'))
    rospy.Subscriber('/r3/pose', Odometry, poseCallbackWrapper('r3'))
    rospy.Subscriber('/r4/pose', Odometry, poseCallbackWrapper('r4'))
    rospy.Subscriber('/r5/pose', Odometry, poseCallbackWrapper('r5'))

    rospy.spin()
