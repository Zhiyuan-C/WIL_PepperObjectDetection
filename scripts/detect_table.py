#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan


import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

# roslaunch pepper_dcm_bringup pepper_bringup.launch network_interface:=enp2s0 roscore_ip:=kate-iMac.local
# roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.0.139 network_interface:=enp2s0 roscore_ip:=kate-iMac.local
# roslaunch wil_pepper_object_detection moveit_planner.launch
# rosrun find_object_2d find_object_2d image:=/naoqi_driver/camera/front/image_raw
# rosrun moveit_commander moveit_commander_cmdline.py

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
        rospy.Subscriber('/laser/srd_front/scan', LaserScan, self.get_laser_msg)
        # initialise publisher
        pub_msg = rospy.Publisher('detect_table_result', String, queue_size=10)
        pub_approach_msg = rospy.Publisher('approach_table', String, queue_size=10)
        rate = rospy.Rate(10)
        
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
        self.at_front = False
        self.detect_object = False
        self.finish_one_side = False
        self.finish_position_check = False
        self.finish_detect = False
        self.left_right_check_count = 4
        self.up_down_check_count = 3
        self.cb_count = 0

        self.far = False

        rospy.loginfo("Start detecting table")
        while not rospy.is_shutdown(): 
            
            # rospy.loginfo(self.detect_object)
            if not self.finish_detect:
                self.pitch_check()
            elif self.finish_detect:
                if self.at_front:
                    rospy.loginfo("The object is in front of pepper!")
                    if self.far:
                        pub_approach_msg.publish("ready")
                    # move close to the table object
                elif self.at_left:
                    pub_msg.publish("left")

                elif self.at_right:
                    pub_msg.publish("right")

            elif self.finish_one_side:
                rospy.loginfo("No object in this direction, turn around")        

                # rospy.loginfo("======Object detected, object center is======")
                # rospy.loginfo(self.table_center)
                # rospy.loginfo(type(self.table_center))
            # 2, string, publish when table data detected
            #     pub_msg(self.pub_data)

            rate.sleep()


    def pitch_check(self):
        """Move the Pepper's head up and down to detect object"""
        pitch_val = 1.0
        count = 0
        rate = rospy.Rate(10)
        time.sleep(5)
        if not self.detect_object:
            for i in range(3):
                if rospy.is_shutdown():
                    break
                if self.finish_detect:
                    break
                count = i + 1
                rospy.loginfo("====Start position checking %s ====" % count)
                pitch_val -= 0.5
                self.move_head(0.0, pitch_val)
                time.sleep(2)
                if self.detect_object:
                    rospy.loginfo("object detected at joint val => %s" % self.joint_goal)
                    self.get_detected_dirction(self.joint_goal)
                    self.finish_detect = True
                    time.sleep(2)
                    self.move_head(0.0, pitch_val)
                    time.sleep(2)
                    break
                else:
                    self.yaw_check(pitch_val)
        else:
            self.finish_detect = True
            self.at_front = True
            
        if not self.detect_object and not self.finish_one_side and not self.finish_detect:
            rospy.loginfo("====No object in this side, back to initial====")
            self.finish_one_side = True
            self.move_head(0.0, 0.0)
            time.sleep(2)

    def yaw_check(self, pitch_val):
        """Move Pepper's head left and right to detect object"""
        right = False
        val = 0.0
        change_yaw_val = 0.5
        for i in range(4):
            if rospy.is_shutdown():
                break
            if self.detect_object:
                rospy.loginfo("object detected at joint val => %s" % self.joint_goal)
                self.get_detected_dirction(self.joint_goal)
                self.finish_detect = True
                #/////////////
                # move pitch_val up a bit when its looking down, 
                # because if detect at very side, the head slitly went up, 
                # if stay at the original position, when turning will not detect object
                if self.joint_goal[1] == 0.5 and (self.joint_goal[0] == 1.0 or self.joint_goal[0] == -1.0):
                    pitch_val = 0.1
                self.move_head(0.0, pitch_val)
                time.sleep(2)
                break
            else:
                if right:
                    change_yaw_val = -0.5
                cr = i + 1
                rospy.loginfo("====Start side checking %s ====" % cr)
                val += change_yaw_val
                self.move_head(val, pitch_val)
                if val == 1.0:
                    val = 0.0
                    right = True
    
    def move_head(self, yaw_val, pitch_val):
        """ Set joints value for move the head of Pepper
        Argument:
            yaw_val: float value for yaw joint (move left or right)
            pitch_val: float value for pitch joint (move up or down)
        """
        if (-0.6 <= pitch_val <= 0.6) and (-1.5 <= yaw_val <= 1.5):
            self.joint_goal[0] = yaw_val
            self.joint_goal[1] = pitch_val # move down
            self.execute_joint_goal()
            # when at 0.5, the max left and right are 1 , -1
            # rospy.loginfo(move_group.get_current_state())
            rospy.loginfo("moved to => %s" % self.joint_goal)
            rospy.loginfo("Sleep for 3 sec")
            time.sleep(3)
        else:
            raise Error("Joint value out of range")

    def get_detected_dirction(self, joint_val):
        if joint_val[0] > 0:
            self.at_left = True
        elif joint_val[0] < 0:
            self.at_right = True
        elif joint_val[0] == 0:
            self.at_front = True

    def execute_joint_goal(self):
        """ Execute joint value to let Pepper move its head """
        self.move_group.go(self.joint_goal, wait=True)
        self.move_group.stop()

    def detect_table(self, objects):
        if len(objects.data) > 0 and objects.data[0] == 1:
            self.detect_object = True
        else:
            self.detect_object = False
    
    def get_laser_msg(self, msg):
        if msg.ranges[7] > 0.5:
            self.far = True



if __name__ == '__main__':
    detect = DetectTable()

    