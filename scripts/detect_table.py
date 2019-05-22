#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

class Error(Exception):
    def __init__(self, error_message):
        self.error_message = error_message
    def ___str__(self):
        return repr(self.error_message)

# spin pepper to detect table
class DetectTable(object):
    def __init__(self):
        # initialise
        rospy.init_node("detect_table", anonymous=True)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        # initialise publisher
        # pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # pub_msg = rospy.Publisher('detect/table/result', String, queue_size=10)

        rate = rospy.Rate(10)
        
        rospy.loginfo("Start detecting table")

        # initialise moveit
        super(DetectTable, self).__init__()
        # moveit_commander.roscpp_initialize(sys.argv)
        self.pepper = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("head")
        self.joint_goal = self.move_group.get_current_joint_values()

        # initialise for object recognition
        # self.finish_one_side = False
        self.at_left = False
        self.at_right = False
        self.at_center = False
        self.detect_object = False
        self.finish_one_side = False
        self.finish_position_check = False
        self.left_right_check_count = 4
        self.up_down_check_count = 3
        self.cb_count = 0
        while not rospy.is_shutdown():
            self.one_side()
            rate.sleep 
        # initialise for turning
        # self.spin_pepper = Twist()
        # self.start_spin = False
        # self.already_spined = False
        # stop_pub_vel = False
        # while not rospy.is_shutdown():
        #     # 1, velocity, when table data is not detect, publish to the velocity
        #     if not self.already_spined:
        #         pub_vel(self.spin_pepper) # check if last message published
        #     elif self.already_spined:
        #         pub_vel(self.spin_pepper)
        #         break
        #     # 2, string, publish when table data detected
        #     pub_msg(self.pub_data)

        #     rate.sleep()

    def one_side(self):
        if self.up_down_check_count == 3 and not self.detect_object:
            rospy.loginfo("====Start initial position checking====")
            self.move_head(0.0, 0.5)
            self.up_down_check_count -= 1
            time.sleep(2)
            self.left_right_check_count = 4
            self.left_right(0.5)
        elif self.up_down_check_count == 2 and not self.detect_object:
            rospy.loginfo("====Start second position checking====")
            self.move_head(0.0, 0.0)
            self.up_down_check_count -= 1
            time.sleep(2)
            self.left_right_check_count = 4
            self.left_right(0.0)
        elif self.up_down_check_count == 1 and not self.detect_object:
            rospy.loginfo("====Start final position checking====")
            self.move_head(0.0, -0.5)
            self.up_down_check_count -= 1
            time.sleep(2)
            self.left_right_check_count = 4
            self.left_right(-0.5)
        elif self.up_down_check_count == 0 and not self.detect_object:
            rospy.loginfo("====No object in this side, back to initial====")
            self.move_head(0.0, 0.0)
            self.finish_one_side = True
            time.sleep(2)
   

    def left_right(self, pitch_val):
        rospy.loginfo("current object detect => %s " % self.detect_object)
        self.cb_count += 1
        rospy.loginfo("call back count => %s " % self.cb_count)    
        if self.left_right_check_count == 4 and not self.detect_object:
            rospy.loginfo("====Start left checking 1====")
            move_head(self, 0.5, pitch_val)
            self.left_right_check_count -= 1
        elif self.left_right_check_count == 3 and not self.detect_object:
            rospy.loginfo("====Start left checking 2====")
            move_head(self, 1.0, pitch_val)
            self.left_right_check_count -= 1
        elif self.left_right_check_count == 2 and not self.detect_object:
            rospy.loginfo("====Start right checking 1====")
            move_head(self, -0.5, pitch_val)
            self.left_right_check_count -= 1
        elif self.left_right_check_count == 1 and not self.detect_object:
            rospy.loginfo("====Start right checking 2====")
            move_head(self, -1.0, pitch_val)
            self.left_right_check_count -= 1
        elif self.detect_object:
            rospy.loginfo("object detected at joint val => %s" % self.joint_goal)
    

    def move_head(self, yaw_val, pitch_val):
        if -0.6 <= self.joint_goal[1] <= 0.6 and if -1.5 <= self.joint_goal[0] <= 1.5:
            self.joint_goal[0] = yaw_val
            self.joint_goal[1] = pitch_val # move down
            # self.execute_joint_goal()
            # when at 0.5, the max left and right are 1 , -1
            # rospy.loginfo(move_group.get_current_state())
            rospy.loginfo("moved to => %s" % self.joint_goal)
            rospy.loginfo("Sleep for 3 sec")
            time.sleep(3)
        else:
            raise Error("Joint value out of range")

    def execute_joint_goal(self):
        self.move_group.go(self.joint_goal, wait=True)
        self.move_group.stop()


    def turning_pepper(self, val):
        # after spin when one side no table object detect, change one side detect to false
        if self.detected_table:
            self.spin_pepper.angular.z = val
            self.start_spin = True
        elif objects.data[0] == 1:
            self.spin_pepper.angular.z = 0.0
            if self.start_spin:
                self.already_spined = True

    def detect_table(self, objects):
        if len(objects.data) > 0 and objects.data[0] == 1:
            self.detect_object = True
        else:
            self.detect_object = False

        # if not self.finish_one_side:
        #     self.move_head_detect_tb()
        # self.turning_pepper(detected_table) 
        
        
        
        # more
        # check if the object width and height is in relation to the real frame
        # if is in relation to the real frame
        # spin the pepper, until the object is in the ejge of the camera

if __name__ == '__main__':
    detect = DetectTable()
    