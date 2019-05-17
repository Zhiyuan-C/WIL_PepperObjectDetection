#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
import cv2
import numpy

def call_back(objects):
    if len(objects.data) == 0:
        rospy.loginfo("no data")
    else:
        if objects.data[0] == 1:
            mat = numpy.zeros((3, 3), dtype='float32')
            mat[0, 0] = objects.data[3]
            mat[1, 0] = objects.data[4]
            mat[2, 0] = objects.data[5]
            mat[0, 1] = objects.data[6]
            mat[1, 1] = objects.data[7]
            mat[2, 1] = objects.data[8]
            mat[0, 2] = objects.data[9]
            mat[1, 2] = objects.data[10]
            mat[2, 2] = objects.data[11]
            
            width = objects.data[1]
            height = objects.data[2]

            inpt_array = numpy.float32([[0,0],[width-1,0],[0,height-1],[width-1,height-1]]).reshape(-1,1,2)

            outpt_array = cv2.perspectiveTransform(inpt_array, mat)

            x_pos = (outpt_array[0, 0] + outpt_array[1, 0] + outpt_array[2, 0] + outpt_array[3, 0]) / 4

            rospy.loginfo(inpt_array)
            rospy.loginfo(x_pos)
    

def main():
    rospy.init_node('corner_experiment', anonymous=True)
    rospy.Subscriber("/objects", Float32MultiArray, call_back)
    rospy.spin()

if __name__ == '__main__':
    main()



