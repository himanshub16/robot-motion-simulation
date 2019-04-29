#!/usr/bin/env python

import math
import tf
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

ROBOTS = ['r1', 'r2', 'r3', 'r4']
# CUR_POS = {rname: [0, 0, 0] for rname in ROBOTS}
CUR_POS = {
    'r1': [0, 0, 0],
    'r2': [0, 10, 0],
    'r3': [10, 10, 0],
    'r4': [10, 0, 0],
}

GOAL_MAP = {
    'r1': [10, 10],
    'r2': [10, 0],
    'r3': [0, 0],
    'r4': [0, 10],
}
pubs = {}


def calculate_angle_for_collision_avoidance(rname):
    global GOAL_MAP, CUR_POS
    r_goal = [GOAL_MAP[rname][0] - CUR_POS[rname][0],
              GOAL_MAP[rname][1] - CUR_POS[rname][1]]

    r_others = [
        [CUR_POS[r][0]-CUR_POS[rname][0],
         CUR_POS[r][1]-CUR_POS[rname][1]]
        for r in ROBOTS if r != rname
    ]
    # 0.001 added to avoid ZeroDivisionError
    dist_goal = math.hypot(*r_goal) + 0.001
    dist_others = [math.hypot(*r)+0.001 for r in r_others]

    # calculating force in 2 dimension
    k_goal = 10
    k_others = -2

    F_x = k_goal * r_goal[0] / (dist_goal**2) + \
          sum(k_others * r[0] / (d**2) for r, d in zip(r_others, dist_others))

    F_y = k_goal * r_goal[1] / (dist_goal**2) + \
          sum(k_others * r[1] / (d**2) for r, d in zip(r_others, dist_others))

    angle = math.atan2(F_y, F_x)
    goal_reached = dist_goal < 1
    return angle, goal_reached


def moveRobot(rname):
    global pubs, CUR_POS
    listener.waitForTransform('/world', '/{}_base_link'.format(rname),
                                rospy.Time(0), rospy.Duration(5))
    trans, rot = listener.lookupTransform('/world', '/{}_base_link'.format(rname),
                                            rospy.Time(0))
    euler = euler_from_quaternion(rot)
    CUR_POS[rname] = [trans[0], trans[1], euler[2]]

    angle, goal_reached = calculate_angle_for_collision_avoidance(rname)
    theta = angle - euler[2]
    fixed_angle = math.atan2(math.sin(theta), math.cos(theta))

    vel = Twist()
    vel.linear.x = 1 if not goal_reached else 0
    vel.angular.z = math.copysign(min(abs(fixed_angle), 2), fixed_angle)

    pubs[rname].publish(vel)


def poseCallbackWrapper(rname):
    def callback(data):
        moveRobot(rname)
    return callback


if __name__ == '__main__':
    rospy.init_node('move_em')
    listener = tf.TransformListener(rospy.Time(100))

    for rname in ROBOTS:
        pubs[rname] = rospy.Publisher('/{}/cmd_vel'.format(rname), Twist, queue_size=10)

    rate = rospy.Rate(10)
    rospy.loginfo('starting motion')
    try:
        while not rospy.is_shutdown():
            for rname in ROBOTS:
                moveRobot(rname)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('shutdown')

