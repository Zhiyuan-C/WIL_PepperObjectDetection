#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# spin pepper to detect table
class DetectTable(object):
    def __init__(self):
        rospy.init_node("detect/table", anonymous=True)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        pub_msg = rospy.Publisher('detect/table/result', String, queue_size=10)
        rospy.Rate = (10)
        self.table_detected = False
        self.spin_pepper = Twist()
        self.pub_data = 'false'
        rospy.loginfo("Start detecting table")
        # publish two nodes
        while not self.table_detected and not rospy.is_shutdown():
            pub_vel(self.spin_pepper)
            rate.sleep()
        while not rospy.is_shutdown():
            pub_msg(self.pub_data)


            
        # 1, velocity, when table data is not detect, publish to the velocity
        # 2, string, publish when table data detected
        

# once table object find, stop spin, break loop
# subscribe to 1 node
