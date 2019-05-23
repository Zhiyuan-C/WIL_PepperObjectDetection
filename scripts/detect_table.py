#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

class Error(Exception):
    """Customise error message for debug"""
    def __init__(self, error_message):
        """ Initialise class """
        self.error_message = error_message
    def ___str__(self):
        """ Print the error message to the terminal """
        return repr(self.error_message)

# spin pepper to detect table
class DetectTable(object):
    def __init__(self):
        """ Initialise class, and looping the code untill shutdown the node """
        # initialise node and subscriber
        rospy.init_node("detect_table", anonymous=True)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        rospy.Subscriber('/laser/srd_front/scan', LaserScan, self.get_laser_msg)
        # initialise publisher
        pub_msg = rospy.Publisher('detect_table_result', String, queue_size=10)
        pub_approach_msg = rospy.Publisher('approach_table', String, queue_size=10)
        rate = rospy.Rate(10)
        
        # initialise moveit
        super(DetectTable, self).__init__()
        self.pepper = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("head")
        self.joint_goal = self.move_group.get_current_joint_values()

        # initialise for object recognition
        self.at_left = False
        self.at_right = False
        self.at_front = False
        self.detect_object = False
        self.finish_one_side = False
        self.finish_position_check = False
        self.finish_detect = False
        self.left_right_check_count = 4
        self.up_down_check_count = 3
        self.far = False

        # start node with call back functions and publish new topics
        rospy.loginfo("******Start to detecting table object******")
        while not rospy.is_shutdown():
            # move pepper's head to detect object if haven't move
            if not self.finish_detect:
                self.pitch_check()
            elif self.finish_detect:
                # publish message to approach topic if pepper is far from object
                if self.at_front:
                    if self.far:
                        pub_approach_msg.publish("ready")

                # publish message to detect_table_result topic if object is at either left or right side of pepper
                elif self.at_left:
                    pub_msg.publish("left")
                elif self.at_right:
                    pub_msg.publish("right")

                # log message onto terminal if finish move head still no object detect
                elif self.finish_one_side:
                    rospy.loginfo("====No object detected====")
                    break        
            rate.sleep()

    def pitch_check(self):
        """Move the Pepper's head up and down to detect object"""
        pitch_val = 1.0
        count = 0
        time.sleep(5) # pause the function, allow the detect_table function start to work before move pepper's head
        if not self.detect_object:
            for i in range(3):
                # break the loop when user press ctrl+c
                if rospy.is_shutdown():
                    break
                if self.finish_detect:
                    break
                count = i + 1
                rospy.loginfo("====Start detecting object at position %s ====" % count)
                pitch_val -= 0.5 # start at 0.5 (looking down), then move to -0.5 (looking up)
                self.move_head(0.0, pitch_val)
                rospy.loginfo("Searching for object")
                time.sleep(5)
                if self.detect_object:
                    self.get_detected_dirction(self.joint_goal)
                    self.finish_detect = True
                    time.sleep(2)
                    self.move_head(0.0, pitch_val)
                    time.sleep(5)
                    break
                else:
                    # if no object detect when move up or down, then move left and right to detect
                    self.yaw_check(pitch_val)
                if count == 3 and not self.detect_object:
                    # if at last round of the loop still no object detect, change the flag to true so it will stop move
                    self.finish_one_side = True
                    self.finish_detect = True
                    self.move_head(0.0, 0.0)
                    time.sleep(3)
        elif self.detect_object:
            # if there is object detect before move the head, which means the object is at front of pepper
            self.finish_detect = True
            self.at_front = True    

    def yaw_check(self, pitch_val):
        """Move Pepper's head left and right to detect object"""
        right = False
        val = 0.0
        change_yaw_val = 0.5 # positive means move left direction, negative means move right direction
        for i in range(4):
            if rospy.is_shutdown():
                break
            if self.detect_object:
                self.get_detected_dirction(self.joint_goal)
                self.finish_detect = True
                #/////////////
                # move pitch_val up a bit when its looking down, 
                # because if detect at very side, the head slitly went up, 
                # if stay at the original position, when turning will not detect object
                if self.joint_goal[1] == 0.5 and (self.joint_goal[0] == 1.0 or self.joint_goal[0] == -1.0):
                    pitch_val = 0.1
                self.move_head(0.0, pitch_val)
                time.sleep(5)
                break
            else:
                # if right flag is true, then means now pepper will detect right side, the change value will be set to negative
                if right:
                    # if pepper is looking down, then pepper will not be able to move very right, so reduce the change value
                    if pitch_val == 0.5:
                        change_yaw_val = -0.3
                    else:
                        change_yaw_val = -0.5
                cr = i + 1
                rospy.loginfo("====Start detecting side view at position %s ====" % cr)
                val += change_yaw_val
                self.move_head(val, pitch_val)
                rospy.loginfo("Searching for object")
                time.sleep(5)
                # if val is 1.0 which means pepper finish left side, initialise val to 0 and change right side flag to true
                if val == 1.0:
                    val = 0.0
                    right = True
        # make sure after last round, if pepper detect any object, change finish_detect flag to true so it will not continue moving
        # this can be changed inside the for loop similar to the for loop in pitch_check function.
        if self.detect_object:
            self.get_detected_dirction(self.joint_goal)
            self.finish_detect = True
            #/////////////
            # move pitch_val up a bit when its looking down, 
            # because if detect at very side, the head slitly went up, 
            # if stay at the original position, when turning will not detect object
            if self.joint_goal[1] == 0.5 and (self.joint_goal[0] == 1.0 or self.joint_goal[0] == -1.0):
                pitch_val = 0.1
            self.move_head(0.0, pitch_val)
            time.sleep(5)
    
    def move_head(self, yaw_val, pitch_val):
        """ Set joints value for move the head of Pepper
        Argument:
            yaw_val: float value for yaw joint (move left or right)
            pitch_val: float value for pitch joint (move up or down)
        """
        if (-0.6 <= pitch_val <= 0.6) and (-1.5 <= yaw_val <= 1.5):
            self.joint_goal[0] = yaw_val # move left / right
            self.joint_goal[1] = pitch_val # move up / down
            self.execute_joint_goal()
        else:
            raise Error("Joint value out of range")

    def get_detected_dirction(self, joint_val):
        """ According to the joint value, determine which side is the object at
        Argument:
            joint_val: joint value of current pepper's head position
        """
        if joint_val[0] > 0:
            rospy.loginfo("Object detected, is at left side of Pepper")
            rospy.loginfo("======Pepper is about to turn towards left======")
            self.at_left = True
        elif joint_val[0] < 0:
            rospy.loginfo("Object detected, is at right side of Pepper")
            rospy.loginfo("======Pepper is about to turn towards right======")
            self.at_right = True
        elif joint_val[0] == 0:
            self.at_front = True
            rospy.loginfo("Object detected, is in front of Pepper")

    def execute_joint_goal(self):
        """ Execute joint value to let Pepper move its head """
        self.move_group.go(self.joint_goal, wait=True)
        self.move_group.stop()

    def detect_table(self, objects):
        """ Change the flag of detected object depend on if the table object is detected or not """
        if len(objects.data) > 0 and objects.data[0] == 1:
            self.detect_object = True
        else:
            self.detect_object = False
    
    def get_laser_msg(self, msg):
        """ Check current laser message to determine if the table is far from pepper """
        if msg.ranges[7] > 0.5:
            self.far = True

if __name__ == '__main__':
    detect = DetectTable()

    