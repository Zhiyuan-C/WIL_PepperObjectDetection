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

def object_detection(data):
    pepper = moveit_commander.RobotCommander()
    # move head
    move_group = moveit_commander.MoveGroupCommander("head")
    joint_goal = move_group.get_current_joint_values()

    # if no object detected
    if data.data.count() == 0:
        pass
    elif data.data.count() > 0:
        object_id = data.data[0]
        
    
def main():
    # initialise ros node
    rospy.init_node("object_detection", anonymous=True)
    rospy.Subscriber("object", Float32MultiArray, object_detection, queue_size=1)

if __name__ == '__main__':
    main()