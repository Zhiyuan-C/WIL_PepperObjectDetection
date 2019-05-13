#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("spin_pepper")
    sub = rospy.Subscriber("/pepper_robot/odom", Odometry, call_back) # check the type with pepper, rosmsg show Odometry
