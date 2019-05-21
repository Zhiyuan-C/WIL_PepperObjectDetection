#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

# spin pepper to detect table
class DetectTable(object):
    def __init__(self):
        # initialise
        rospy.init_node("detect/table", anonymous=True)
        rospy.Subscriber("/objects", Float32MultiArray, self.detect_table)
        rospy.Subscriber("/objects", Float32MultiArray, self.move_head_detect_tb)
        # initialise publisher
        pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        pub_msg = rospy.Publisher('detect/table/result', String, queue_size=10)

        rospy.Rate = (10)
        
        rospy.loginfo("Start detecting table")

        # initialise moveit
        super(MotionPlan, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.pepper = moveit_commander.RobotCommander()

        self.spin_pepper = Twist()
        stop_pub_vel = False
        # publish two nodes
        self.start_spin = False
        self.already_spined = False
        while not rospy.is_shutdown():
            # 1, velocity, when table data is not detect, publish to the velocity
            if not self.already_spined:
                pub_vel(self.spin_pepper) # check if last message published
            elif self.already_spined:
                pub_vel(self.spin_pepper)
                break
            # 2, string, publish when table data detected
            pub_msg(self.pub_data)

            rate.sleep()
        

    def move_head_detect_tb(self, objects):
        move_group = moveit_commander.MoveGroupCommander("head")
        joint_goal = move_group.get_current_joint_values()
        #[0] to move left or right
        #[1] to move up or down
        #set initial joint
        joint_goal[0] = 0.0
        joint_goal[1] = 0.5 # move down
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        count = 0
        while count < 5:
            if self.
        #move right

    def turning_pepper(self):
        if len(objects.data) == 0:
            self.spin_pepper.angular.z = 0.1
            self.start_spin = True
        elif objects.data[0] == 1:
            self.spin_pepper.angular.z = 0.0
            if self.start_spin:
                self.already_spined = True

    def detect_table(self, objects):
        # no object detect
        
        
        # more
        # check if the object width and height is in relation to the real frame
        # if is in relation to the real frame
        # spin the pepper, until the object is in the ejge of the camera

if __name__ == '__main__':
    detect = DetectTable()