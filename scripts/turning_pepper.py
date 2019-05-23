#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

class TurningPepper(object):


    def get_object_center(self, objects):
        if len(objects.data) > 0 and objects.data[0] == 1:
            self.detect_object = True
            # get transformation matrix
            matrix = numpy.zeros((3, 3), dtype='float32')
            matrix[0, 0] = objects.data[3]
            matrix[1, 0] = objects.data[4]
            matrix[2, 0] = objects.data[5]
            matrix[0, 1] = objects.data[6]
            matrix[1, 1] = objects.data[7]
            matrix[2, 1] = objects.data[8]
            matrix[0, 2] = objects.data[9]
            matrix[1, 2] = objects.data[10]
            matrix[2, 2] = objects.data[11]
            
            # get array of 2d vectors to transform
            width = objects.data[1]
            height = objects.data[2]
            inpt_array = numpy.float32([[0,0],[width-1,0],[0,height-1],[width-1,height-1]]).reshape(-1,1,2)
            
            # perfrom perspective transformation
            outpt_array = cv2.perspectiveTransform(inpt_array, matrix)
            
            # get object center
            self.table_center = (outpt_array[0, 0] + outpt_array[1, 0] + outpt_array[2, 0] + outpt_array[3, 0]) / 4

    def get_direction(self, msg):
        pass


if __name__ == "__main__":
    start = TurningPepper()