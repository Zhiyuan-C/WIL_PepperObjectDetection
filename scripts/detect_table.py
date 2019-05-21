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
        self.spin_pepper = Twist()
        rospy.loginfo("Start detecting table")
        stop_pub_vel = False
        # publish two nodes
        self.start_spin = False
        self.already_spined = False
        while not rospy.is_shutdown():
            # 1, velocity, when table data is not detect, publish to the velocity
            if not self.already_spined:
                pub_vel(self.spin_pepper) # check if last message published
            elif self.already_spined:
                pub_vel(self.spin_pepper)
                break
            # 2, string, publish when table data detected
            pub_msg(self.pub_data)

            rate.sleep()
        
    
    def detect_table(objects):
        # no object detect
        if len(objects.data) == 0:
            self.spin_pepper.angular.z = 0.1
            self.start_spin = True
        elif objects.data[0] == 1:
            self.spin_pepper.angular.z = 0.0
            if self.start_spin:
                self.already_spined = True
        
        # more
        # check if the object width and height is in relation to the real frame
        # if is in relation to the real frame
        # spin the pepper, until the object is in the ejge of the camera

if __name__ == '__main__':
    detect = DetectTable()