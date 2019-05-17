#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
import cv2
import numpy

def call_back(objects):
    if objects.data[0] == 1:
        mat = numpy.zeros((3, 3), dtype='float32')
        mat[0, 0] = object.data[3]
        mat[1, 0] = object.data[4]
        mat[2, 0] = object.data[5]
        mat[0, 1] = object.data[6]
        mat[1, 1] = object.data[7]
        mat[2, 1] = object.data[8]
        mat[0, 2] = object.data[9]
        mat[1, 2] = object.data[10]
        mat[2, 2] = object.data[11]

