#!/usr/bin/env python

import rospy
from sensor_msgs import LaserScan # check with pepper, rostopic type /pepper_robot/laser
from geometry_msgs import Twist

def call_back(msg):
    print(msg.ranges[60])
    move.linear.x = 0.1
    if msg.ranges[60] < 1:
        move.linear.x = 0
    pub.publish(move)

