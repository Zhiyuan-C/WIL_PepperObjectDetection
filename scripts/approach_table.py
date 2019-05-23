#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ApproachTable(object):
    """ Approach to table when is far from table """
    def __init__(self):
        """ Initialise class, and looping the code untill shutdown the node """
        # initialise
        rospy.init_node("approach", anonymous=True)
        rospy.Subscriber('/laser/srd_front/scan', LaserScan, self.get_laser_msg)
        rospy.Subscriber('approach_table', String, self.get_approach_info)
        # initialise publisher
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # initialise for approaching table
        self.approach = False
        self.move = Twist()
        self.close_to = False
        # start node with call back functions and publish new topics
        while not rospy.is_shutdown():
            if self.approach:
                pub.publish(self.move)
                # publish message untill is close to the table, then after 5 seconds stop node
                if self.close_to:
                    rospy.loginfo("Object is approached")
                    time.sleep(5)
                    break

    def get_laser_msg(self, msg):
    """ Determine if pepper should move or not accoding to the laser message """
        if msg.ranges[7] <= 0.5: # should stop move forward if the middle value is less than 4, check first with actual no move
            self.close_to = True
            self.move.linear.x = 0.0
        elif msg.ranges[7] >= 0.5:
            self.move.linear.x = 0.1
    
    def get_approach_info(self, msg):
    """ Determine if pepper is ready to approach or not """
        if len(msg.data) > 0 and msg.data == "ready":
            self.approach = True

if __name__ == "__main__":
    start = ApproachTable()