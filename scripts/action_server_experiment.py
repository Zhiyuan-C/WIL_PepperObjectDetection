#! /usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult

class ActionServerExperiment():
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("move_head", FollowJointTrajectoryAction,execute_cb=self.call_back, auto_start=False)
        self.action_server.start()
        

if __name__ == '__main__':
    rospy.init_node("action_server_experiment")
    s = ActionServerExperiment()
    rospy.spin()