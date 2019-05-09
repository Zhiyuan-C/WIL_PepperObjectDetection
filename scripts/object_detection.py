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


class MotionPlan(object):
    """Motion plan basied on object detection"""
    def __init__(self):
        """Initialisation"""
        super(MotionPlan, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        
        pepper = moveit_commander.RobotCommander()
        self.pepper = pepper
        self.sub = rospy.Subscriber("/objects", Float32MultiArray, self.object_recognition)
    
    def object_recognition(self, objects):
        # if no object detected
        object_data_array = []
        if objects.data.count() == 0:
            return object_data_array
        elif objects.data.count() > 0:
            object_data_array = objects.data
            return object_data_array 


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