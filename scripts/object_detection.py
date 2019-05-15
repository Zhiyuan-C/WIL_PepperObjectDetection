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
        self.detecting_table = False
        self.approaching = False
        self.detected_object = False
        self.spin = Twist()
        self.approach = Twist()
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_object)
        rospy.Subscriber('/laser/srd_front/scan', LaserScan, self.approach_table)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            if detecting_table:
                pub.publish(self.spin)
            if approaching:
                pub.publish(self.approach)
                self.approaching = False
            rate.sleep()
    
    def detect_table(self, objects):
        """Detection for table with special object, id = 1"""
        if objects.data.count() == 0:
            self.spin.angular.z = 0.1
            self.detecting_table = True
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
        # check again with pepper to see if the data published is in order
        # the code here is assume is in order, if not in order then modify the code before testing
        object_name = ""
        if len(objects.data) > 0:
            if objects.data[0] == 1.0 and len(objects.data) > 12:
                # if the table is detected, then the object data array should be greater than 12 so the second object is also detected
                # check frame
                if objects.data[13] == 2.0:
                    object_name = "tea1"
                    print("detected {}".format(object_name))
                elif objects.data[13] == 3.0:
                    object_name = "tea2"
                    print("detected {}".format(object_name))
                elif objects.data[13] == 4.0:
                    object_name = "tea3"
                    print("detected {}".format(object_name))
                elif objects.data[13] == 5.0:
                    object_name = "tea4"
                    print("detected {}".format(object_name))
                elif objects.data[13] == 6.0:
                    object_name = "tea5"
                    print("detected {}".format(object_name))
                self.detected_object = True
        elif len(objects.data) == 0 and not detecting_table:
            # if there is no object detected, and is not detecting for table object, then move the head to detect
            print("no object detected, start searching")
            self.detected_object = False
            self.move_head()
                    
    
    def move_hd_to_right(self, joint_goal):
        joint_goal[0] = 0.0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        # move to right
        pass
    
    def move_hd_to_left(self):
        # move to left
        pass
    
    def move_first_pos(self):
        # move head in first position looking down, detect object, then move left and right, detect object
        move_group = moveit_commander.MoveGroupCommander("head")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = 0.5 # move down
        print(self.detected_object)
        if self.detected_object:
            return True
        for i in range(2):
            joint_goal[0] += 0.8
            if self.detected_object:
                break
        if not self.detected_object:
            joint_goal[0] = 0.0
        for i in range(2):
            joint_goal[0] -= 0.8
            if self.detected_object:
                break
    
    def move_second_pos(self):
        # move head in second position looking normal, detect object, then move left and right, detect object
        pass

    def move_third_pos(self):
        pass    
        # move head in third posution looking up, detect object, then move left and right, detect object
    
    def move_head(self):
        searching = True
        self.move_first_pos()
        first_position_done = True
        second_position_done = False
        third_position_done = False
        while searching:
            if self.detected_object:
                searching = False
                # self.detect_object(objects)
                break
            elif first_position_done:
                self.move_second_pos()
                second_position_done = True
                continue
            elif second_position_done:
                self.move_third_pos()
                third_position_done = True
                continue
            else:
                searching = False
                break



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