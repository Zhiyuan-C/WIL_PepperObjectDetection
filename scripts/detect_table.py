#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# spin pepper to detect table
# once table object find, stop spin, break loop
# subscribe to 1 node
# publish two nodes
# 1, velocity, when table data is not detect, publish to the velocity
# 2, string, publish when table data detected