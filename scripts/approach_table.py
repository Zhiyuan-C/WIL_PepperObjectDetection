#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ApproachTable(object):
    def __init__(self):
        
        rospy.init_node("approach_table", anonymous=True)
        rospy.Subscriber('/laser/srd_front/scan', LaserScan, get_laser_msg)
        rospy.Subscriber('approach_table', String, self.get_approach_info)
        pub = rospy.Publisher('cmd_vel', Twist)

        self.approach = False
        self.move = Twist()
        self.close = False

        while not rospy.is_shutdown():
            if self.approach:
                pub.publish(self.move)
                if self.close:
                    sleep(5)
                    break



    def get_laser_msg(msg):
        
        if msg.ranges[7] < 0.45: # should stop move forward if the middle value is less than 4, check first with actual no move
            self.close = True
            self.move.linear.x = 0.0
        elif msg.ranges[7] > 0.45:
            self.move.linear.x = 0.1
    
    def get_approach_info(msg):
        if len(msg.data) > 0 and msg.data == "ready":
            self.approach = True


if __name__ == "__main__":
    start = TurningPepper()