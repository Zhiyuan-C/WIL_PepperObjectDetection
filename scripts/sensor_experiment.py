#!/usr/bin/env python

# this should be performed where pepper does not have any other obstcal surround
# run roslaunch pepper_sensors_py laser.launch
# see http://wiki.ros.org/pepper_sensors_py
# http://doc.aldebaran.com/2-0/family/juliette_technical/laser_juliette.html

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def call_back(msg):
    print("middle: {} || sum: {} || avg: {}".format(msg.ranges[7], sum(msg.ranges), (sum(msg.ranges)/15))) # ranges have 15 messages in total, middle one should be 7, recheck with the srd_front/scan
    move.linear.x = 0.1
    if msg.ranges[7] < 1: # should stop move forward if the middle value is less than 4, check first with actual no move
        move.linear.x = 0
    pub.publish(move)

rospy.init_node("laser_scan")
sub = rospy.Subscriber('/laser/srd_front/scan', LaserScan, call_back)
pub = rospy.Publisher('cmd_vel', Twist)
move = Twist()
rospy.spin()
# def main():
#     rospy.init_node("laser_scan")
#     sub = rospy.Subscriber('/laser/srd_front/scan', LaserScan, call_back)
#     pub = rospy.Publisher('cmd_vel', Twist)
#     move = Twist()

#     rospy.spin()
    

# if __name__ == '__main__':
#     main()