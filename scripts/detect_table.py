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

        # initialise for object recognition
        self.finish_one_side = False
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
        

    def move_head_detect_tb(self, detected_table):
        rospy.loginfo("detected_table => %s" % detected_table)
        rospy.loginfo("start moving head")
        move_group = moveit_commander.MoveGroupCommander("head")
        joint_goal = move_group.get_current_joint_values()
        #[0] to move left or right
        #[1] to move up or down
        #set initial joint
        if not detected_table:
            joint_goal[0] = 0.0
            joint_goal[1] = 0.5 # move down
            # when at 0.5, the max left and right are 1 , -1
            rospy.loginfo("initial joint => %s" % joint_goal)
            # self.execute_joint_goal(joint_goal)
            # rospy.loginfo(move_group.get_current_state())
            time.sleep(3)
            self.move_left_right(detected_table, joint_goal)
        for i in range(2):
            if detected_table:
                break
            else:
                joint_goal[1] -= 0.5
                joint_goal[0] = 0.0
                rospy.loginfo("move up %s time => %s" % (i, joint_goal))
                # self.execute_joint_goal(joint_goal)
                time.sleep(3)
                self.move_left_right(detected_table, joint_goal)
        self.finish_one_side = True
        rospy.loginfo("finish_one_side => %s" % self.finish_one_side)
        
    def move_left_right(self, detected_table, joint_goal):
        rospy.loginfo("finish_one_side => %s" % self.finish_one_side)
        move_left = True
        count = 0
        rospy.loginfo("move_left_right => %s" % detected_table)
        while count < 4:
            if detected_table:
                rospy.loginfo("table detected")
                break
            # the joint value is outside the range
            if 2 < joint_goal[0] < -2:
                rospy.loginfo("out of range")
                break
            # move left
            if move_left:
                joint_goal[0] += 0.5
                rospy.loginfo("move to left => %s" % joint_goal[0])
                time.sleep(3)
                # self.execute_joint_goal(joint_goal)
                if detected_table:
                    joint_goal[0] = 0.0
                    joint_goal[1] = 0.0
                    rospy.loginfo("detected at left, initialise => %s" % joint_goal)
                    time.sleep(5)
                    break
                    # self.execute_joint_goal(joint_goal)
                    #spin_right = True
                # set initialise right position
                if 0.9 <= joint_goal[0] <= 1.2:
                    move_left = False
                    joint_goal[0] = -0.5
                    # self.execute_joint_goal(joint_goal)
                    rospy.loginfo("finish left site, start right side => %s" % joint_goal)
                    time.sleep(3)
                    if detected_table:
                        rospy.loginfo("finish left site, start left side, table detected!")
                        break
            # move right
            else:
                joint_goal[0] -= 0.5
                rospy.loginfo("move to right => %s" % joint_goal[0])
                # self.execute_joint_goal(joint_goal)
                time.sleep(3)

                if detected_table:
                    joint_goal[0] = 0.0
                    joint_goal[1] = 0.0
                    # self.execute_joint_goal(joint_goal)
                    # self.turning_pepper(-0.1)
                    rospy.loginfo("detected at right, initialise => %s" % joint_goal)
                    
                    break
                break
            count += 1
    
    def execute_joint_goal(self, joint_goal):
        move_group.go(joint_goal, wait=True)
        move_group.stop()


    def turning_pepper(self, val):
        # after spin when one side no table object detect, change one side detect to false
        if detected_table:
            self.spin_pepper.angular.z = val
            self.start_spin = True
        elif objects.data[0] == 1:
            self.spin_pepper.angular.z = 0.0
            if self.start_spin:
                self.already_spined = True

    def detect_table(self, objects):
        # no object detect
        detected_table = False
        if len(objects.data) > 0 and objects.data[0] == 1:
            detected_table = True
        
        if not self.finish_one_side:
            self.move_head_detect_tb(detected_table)
        # self.turning_pepper(detected_table) 
        
        
        
        # more
        # check if the object width and height is in relation to the real frame
        # if is in relation to the real frame
        # spin the pepper, until the object is in the ejge of the camera

if __name__ == '__main__':
    detect = DetectTable()