#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ApproachTable(object):
    def __init__(self):
        self.move = Twist()


    def call_back(msg):
        self.move.linear.x = 0.1
        if msg.ranges[7] < 0.45: # should stop move forward if the middle value is less than 4, check first with actual no move
            move.linear.x = 0
