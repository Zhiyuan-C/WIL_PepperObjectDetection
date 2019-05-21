#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
# spin pepper to detect table
class DetectTable(object):
    def __init__(self):
        # initialise
        rospy.init_node("detect_table", anonymous=True)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        # initialise publisher
        # pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # pub_msg = rospy.Publisher('detect/table/result', String, queue_size=10)

        rospy.Rate = (10)
        
        rospy.loginfo("Start detecting table")

        # initialise moveit
        super(DetectTable, self).__init__()
        # moveit_commander.roscpp_initialize(sys.argv)
        self.pepper = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("head")
        self.joint_goal = self.move_group.get_current_joint_values()

        # initialise for object recognition
        # self.finish_one_side = False
        self.initial_pos = True
        self.move_left = False
        self.move_right = False
        self.at_left = False
        self.at_right = False
        self.at_center = False
        self.detect_object = False
        self.checked = False
        self.one_side_check_count = 5
        rospy.spin() # testing purpose, delete this after

        
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
        
    def move_head(self):
        self.joint_goal[0] = 0.5
        rospy.loginfo(self.joint_goal)
        
        if self.one_side_check_count == 5 and not self.detect_object:
            rospy.loginfo("====Start initial position checking====")
            rospy.loginfo("Current => %s" % self.joint_goal)
            self.joint_goal[0] = 0.0
            self.joint_goal[1] = 0.5
            # self.execute_joint_goal()
            self.one_side_check_count -= 1
            rospy.loginfo("Sleep for 3 sec")
            time.sleep(3)
        elif self.one_side_check_count == 4 and not self.detect_object:
            rospy.loginfo("====Start left checking 1====")
            self.move_to_left()
            # self.execute_joint_goal()
            self.one_side_check_count -= 1
            rospy.loginfo("Sleep for 3 sec")
        elif self.one_side_check_count == 3 and not self.detect_object:
            rospy.loginfo("====Start left checking 2====")
            self.move_to_left()
            # self.execute_joint_goal()
            self.one_side_check_count -= 1
            rospy.loginfo("Sleep for 3 sec")
        elif self.one_side_check_count == 2 and not self.detect_object:
            rospy.loginfo("====Start right checking 1====")
            self.joint_goal[0] = -0.5
            self.joint_goal[1] = 0.5
            # self.execute_joint_goal()
            self.one_side_check_count -= 1
        elif self.one_side_check_count == 1 and not self.detect_object:
            rospy.loginfo("====Start right checking 2====")
            self.move_to_right
            # self.execute_joint_goal()
            self.one_side_check_count -= 1
        elif self.one_side_check_count == 0 and not self.detect_object:
            rospy.loginfo("====No object detetect, one side check finish====")
        

            

            
        # pitch_val = 0.5
        # if not self.detect_object and self.initial_pos:
        #     rospy.loginfo(self.detect_object)
        #     # set initial pose
        #     rospy.loginfo("initialise first position")
        #     self.move_center(pitch_val)
        #     self.move_left = True
        #     self.initial_pos = False
        #     self.count += 1
        #     rospy.loginfo("initial => %s" % self.count)
        # elif self.move_left and not self.detect_object:
        #     if self.joint_goal[0] <= 1:
        #         self.move_to_left()
        #         self.count += 1
        #         rospy.loginfo("move left => %s" % self.count)
        # elif self.move_right and not self.detect_object:
        #     if -1 <= self.joint_goal[0]:
        #         self.move_to_right()
        #         self.count += 1
        #         rospy.loginfo("move right => %s" % self.count) 
        # else:
        #     rospy.loginfo("finish move %s" % self.detect_object)

    def move_center(self, pitch_val):
        if -0.5 <= self.joint_goal[1] <= 0.5:
            self.joint_goal[0] = 0.0
            self.joint_goal[1] -= 0.5 # move down
            # when at 0.5, the max left and right are 1 , -1
            # self.execute_joint_goal()
            # rospy.loginfo(move_group.get_current_state())

            time.sleep(3)
            rospy.loginfo("initial joint => %s" % self.joint_goal)

    def move_to_left(self):
        self.joint_goal[0] += 0.5
        # self.execute_joint_goal(joint_goal)
        rospy.loginfo("New => %s" % self.joint_goal)
        if self.joint_goal[0] > 1:
            self.move_left = False
            self.move_right = True
        rospy.loginfo("Sleep for 3 sec")
        time.sleep(3)

    

    def move_to_right(self):
        self.joint_goal[0] -= 0.5
        # self.execute_joint_goal(joint_goal)
        rospy.loginfo("New => %s" % self.joint_goal)
        if self.joint_goal[0] < -1:
            self.move_right = False
        rospy.loginfo("Sleep for 3 sec")
        time.sleep(3)

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
        self.move_head()
        
        # # no object detect
        # if len(objects.data) == 0:
        #     if self.initial_pos:
        #         # set initial pose
        #         rospy.loginfo("initialise first position")
        #         self.joint_goal[0] = 0.0
        #         self.joint_goal[1] = 0.5 # move down
        #         # when at 0.5, the max left and right are 1 , -1
        #         # self.execute_joint_goal(joint_goal)
        #         # rospy.loginfo(move_group.get_current_state())
        #         time.sleep(3)
        #         self.at_center = True
        #         rospy.loginfo("initial joint => %s" % self.joint_goal)
        #         self.move_left = True
        #         self.initial_pos = False
        #     elif self.move_left:
        #         rospy.loginfo("start moving head to left")
        #         self.at_center = False
        #         self.at_left = True
        #         for to_left in range(2):
        #             if len(objects.data) > 0 and objects.data[0] == 1:
        #                 break
        #             self.move_to_left()
        #         self.move_left = False
        #         self.move_right = True
        #     elif self.move_right:
        #         self.at_left = False
        #         self.at_right = True
        #         rospy.loginfo("initialise to right")
        #         self.joint_goal[0] = -0.5
        #         time.sleep(3)
        #         rospy.loginfo("start moving head to right")
        #         for to_right in range(1):
        #             if len(objects.data) > 0 and objects.data[0] == 1:
        #                 break
        #             self.move_to_right()
        #         self.move_right = False
        # elif len(objects.data) > 0 and objects.data[0] == 1:
        #     rospy.loginfo("initial_pos => %s" % self.initial_pos)
        #     rospy.loginfo("at_left => %s" % self.at_left)
        #     rospy.loginfo("at_right => %s" % self.at_right)
        #     rospy.loginfo("at_center => %s" % self.at_center)
                
                

        # while not detected_table:
        #     rospy.loginfo("start moving head")
        #     joint_goal[0] = 0.0
        #     joint_goal[1] = 0.5 # move down
        #     # when at 0.5, the max left and right are 1 , -1
        #     rospy.loginfo("initial joint => %s" % joint_goal)
        #     # self.execute_joint_goal(joint_goal)
        #     # rospy.loginfo(move_group.get_current_state())
        #     time.sleep(3)
        #     self.move_left_right(joint_goal)



        # if not self.finish_one_side:
        #     self.move_head_detect_tb()
        # self.turning_pepper(detected_table) 
        
        
        
        # more
        # check if the object width and height is in relation to the real frame
        # if is in relation to the real frame
        # spin the pepper, until the object is in the ejge of the camera

if __name__ == '__main__':
    detect = DetectTable()