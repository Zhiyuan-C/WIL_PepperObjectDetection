#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

spin = Twist()
detected_obj = False

def call_back(msg):
    if msg.data.count() > 0:
        return True
    else:
        return False

def main():
    rospy.init_node("spin_pepper")
    # sub = rospy.Subscriber("/pepper_robot/odom", Odometry, call_back) # check the type with pepper, rosmsg show Odometry
    sub = rospy.Subscriber("/objects", Float32MultiArray, call_back)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        if detected_obj:
            spin.angular.z = 0.1
        else:
            spin.angular.z = 0.0
        pub.publish(spin)
        rate.sleep()


if __name__ == '__main__':
    main()