#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
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
        stop_pub_vel = False
        # publish two nodes
        # 1, velocity, when table data is not detect, publish to the velocity
        while not stop_pub_vel and not rospy.is_shutdown():
            pub_vel(self.spin_pepper) # check if last message published
            if self.table_detected:
                stop_pub_vel = True
            rate.sleep()
        # 2, string, publish when table data detected
        while not rospy.is_shutdown():
            pub_msg(self.pub_data)
            rate.sleep
    
    def detect_table(objects):
        # no object detect
        if len(objects.data) == 0:
            self.spin_pepper.angular.z = 0.1
        elif objects.data[0] == 1:
            self.spin_pepper.angular.z = 0.0
            self.table_detected = True
            self.pub_data = 'true'
        
        # more
        # check if the object width and height is in relation to the real frame
        # if is in relation to the real frame
        # spin the pepper, until the object is in the ejge of the camera

if __name__ == '__main__':
    detect = DetectTable()