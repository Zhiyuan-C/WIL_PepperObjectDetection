#!/usr/bin/env python

# import the basic libary
import rospy
import sys
import copy
from math import pi
from std_msgs.msg import String, Float32MultiArray

# import for moveit package
import moveit_msgs.msg
import moveit_commander
from moveit_commander.conversions import pose_to_list
# from control_msgs.msg import FollowJointTrajectoryGoal
import geometry_msgs.msg
from geometry_msgs.msg import Twist

# Addition, if time allow
    # spin the base, if possible, test with spin code first
    # make special large object to substute for table (find obj can not recognise object with less feature)
    # search for the special large object, test with sensor code first to see if it can reach to the object
    # with head positin [0,0], spin the base, if special large object detected, spin, face to the object move close to the object
    # then perform the above movement to detect object

class MotionPlan(object):
    """Motion plan basied on object detection"""
    def __init__(self):
        """Initialisation"""
        super(MotionPlan, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
     
        self.pepper = moveit_commander.RobotCommander()
        self.detecting_table = True
        self.approaching = False
        self.spin = Twist()
        self.approach = Twist()
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_object)
        rospy.Subscriber('/pepper/laser/srd_front/scan', LaserScan, self.approach_table)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        while not rospy.is_shutdown():
            if detecting_table:
                pub.publish(self.spin)
            if approaching:
                pub.publish(self.approach)
                self.approaching = False
    
    def detect_table(self, objects):
        """Detection for table with special object, id = 1"""
        if objects.data.count() == 0:
            self.spin.angular.z = 0.1
            print("no table object detected, start searching")
            return "no table object detected, start searching"
        elif objects.data[0] == 1:
            self.spin.angular.z = 0.0
            self.approaching = True
            self.detecting_table = False
            print("Detected table object, stop spin, prepare to approach")
            return "Detected table object, stop spin"
        else:
            self.spin.angular.z = 0.0
            self.approaching = False
            self.detecting_table = False
            print("Object detect, but not table, stop spin, do not approach")
            return "Object detect, but not table, stop spin"  

    def approach_table(self, laser_msg):
        approach.linear.x = 0.1
        if laser_msg.ranges[31] < 4:
            approach.linear.x = 0.0
        
        print("Table detected, start approaching to table")
        return "Start approaching to table"

    def detect_object(self, objects):
        # detect objects
        # if no object detected, move pepper print("no object detect, searching for object")
        pass
    
    def move_hd_to_right(self):
        # move to right
        pass
    
    def move_hd_to_left(self):
        # move to left
        pass
    
    def move_first_pos(self):
        # move head in first position [0, -0.7], detect object, then move left and right, detect object
        pass
    
    def move_second_pos(self):
        # move head in second position [0, 0], detect object, then move left and right, detect object
        pass

    def move_third_pos(self):
        pass    
        # move head in third posution [0, 0.6], detect object, then move left and right, detect object

# https://github.com/ros/ros_comm/blob/ebd9e491e71947889eb81089306698775ab5d2a2/test/test_rospy/test/unit/test_rospy_topics.py


def motion_plan():
    pepper = moveit_commander.RobotCommander()
    # move head
    move_group = moveit_commander.MoveGroupCommander("head")
    joint_goal = move_group.get_current_joint_values()
        
    
def main():
    # initialise ros node
    rospy.init_node("object_detection", anonymous=True)
    

if __name__ == '__main__':
    main()