#!/usr/bin/env python

# this should be performed where pepper does not have any other obstcal surround
# run roslaunch pepper_sensors_py laser.launch

import rospy
from sensor_msgs import LaserScan
from geometry_msgs import Twist

def call_back(msg):
    print(msg.ranges[31]) # ranges have 62 messages in total, middle one should be 31
    move.linear.x = 0.1
    if msg.ranges[31] < 2: # should stop move forward if the middle value is less than 2
        move.linear.x = 0
    pub.publish(move)

def main():
    rospy.init_node("laser_scan")
    sub = rospy.Subscriber('/pepper/laser/srd_front/scan', LaserScan, call_back)
    pub = rospy.Publisher('cmd_vel', Twist)
    move = Twist()

    rospy.spin()

if __name__ == '__main__':
    main()