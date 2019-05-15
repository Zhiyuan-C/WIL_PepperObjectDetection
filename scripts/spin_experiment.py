#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



def call_back(msg):
    # print(len(msg.data))
    if len(msg.data) > 0:
        spin_pepper.angular.z = 0.0
        # detected_obj = True
    else:
        spin_pepper.angular.z = 0.1
        # detected_obj = False
    pub.publish(spin_pepper)

rospy.init_node("spin_pepper")
    # sub = rospy.Subscriber("/pepper_robot/odom", Odometry, call_back) # check the type with pepper, rosmsg show Odometry
sub = rospy.Subscriber("/objects", Float32MultiArray, call_back)
    # print(detected_obj)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
spin_pepper = Twist()

rospy.spin()

# def main():
#     rospy.init_node("spin_pepper")
#     # sub = rospy.Subscriber("/pepper_robot/odom", Odometry, call_back) # check the type with pepper, rosmsg show Odometry
#     sub = rospy.Subscriber("/objects", Float32MultiArray, call_back)
#     # print(detected_obj)
#     pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#     spin_pepper = Twist()

#     rospy.spin()
#     # rate = rospy.Rate(3)
#     # while not rospy.is_shutdown():
#     #     if detected_obj:
#     #         spin.angular.z = 0.1
#     #     else:
#     #         spin.angular.z = 0.0
#     #     pub.publish(spin)
#     #     rate.sleep()


# if __name__ == '__main__':
#     main()