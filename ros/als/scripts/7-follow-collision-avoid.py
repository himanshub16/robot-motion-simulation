#!/usr/bin/env python

import math
import random
import tf
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

ROBOTS = ['r1', 'r2', 'r3', 'r4']
# CUR_POS = {rname: [0, 0, 0] for rname in ROBOTS}
CUR_POS = {
    'r1': [0, 10, 0],
    'r2': [-3, 10, 0],
    'r3': [10, 0, 0],
    'r4': [13, 0, 0],
}

GOALS_r1 = [(0, 10), (5, 10), (10, 10),
             (10, 5), (5, 5), (0, 5),
             (0, 0), (5, 0), (10, 0)]
GOALS_r3 = list(reversed(GOALS_r1))
r1_cur_goal_idx = 0
r3_cur_goal_idx = 0


pubs = {}


def update_goal_map():
    global r1_cur_goal_idx, r3_cur_goal_idx, GOALS_r3, GOALS_r1, CUR_POS
    # r1
    if r1_cur_goal_idx < 8:
        r_goal = [GOALS_r1[r1_cur_goal_idx][0]-CUR_POS['r1'][0],
                  GOALS_r1[r1_cur_goal_idx][1]-CUR_POS['r1'][1]]
        dist_goal = math.hypot(*r_goal)
        if dist_goal < 1:
            r1_cur_goal_idx += 1

    # r2
    # will always follow r1
    # r3
    if r3_cur_goal_idx < 8:
        r_goal = [GOALS_r3[r3_cur_goal_idx][0]-CUR_POS['r3'][0],
                  GOALS_r3[r3_cur_goal_idx][1]-CUR_POS['r3'][1]]
        dist_goal = math.hypot(*r_goal)
        if dist_goal < 1:
            r3_cur_goal_idx += 1

    # r4
    # will always follow r3
    # rospy.loginfo('r1: {}, r2: {}, r3: {}, r4: {}'.format(
    #     GOALS_r1[r1_cur_goal_idx], CUR_POS['r1'],
    #     GOALS_r3[r3_cur_goal_idx], CUR_POS['r3'],
    # ))


def calculate_angle_for_collision_avoidance(rname):
    global r1_cur_goal_idx, r3_cur_goal_idx, GOALS_r3, GOALS_r1, CUR_POS
    update_goal_map()

    if rname == 'r1':
        r_goal = [GOALS_r1[r1_cur_goal_idx][0]-CUR_POS['r1'][0],
                  GOALS_r1[r1_cur_goal_idx][1]-CUR_POS['r1'][1]]
    elif rname == 'r2':
        r_goal = [CUR_POS['r1'][0]-CUR_POS['r2'][0],
                  CUR_POS['r1'][1]-CUR_POS['r2'][1]]
    elif rname == 'r3':
        r_goal = [GOALS_r3[r3_cur_goal_idx][0]-CUR_POS['r3'][0],
                  GOALS_r3[r3_cur_goal_idx][1]-CUR_POS['r3'][1]]
    elif rname == 'r4':
        r_goal = [CUR_POS['r3'][0]-CUR_POS['r4'][0],
                  CUR_POS['r3'][1]-CUR_POS['r4'][1]]
    else:
        return None, True

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

    # introduce random  noise
    F_x += random.random() / 100
    F_y += random.random() / 100

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
