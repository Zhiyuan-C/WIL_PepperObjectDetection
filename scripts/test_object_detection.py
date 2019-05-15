#!/usr/bin/env python

import sys
import unittest
import object_detection
from std_msgs.msg import String, Float32MultiArray

class TestObjectDetection(unittest.TestCase):
    def test_detect_object(self):
        source = object_detection.DetectObject()
        

        
