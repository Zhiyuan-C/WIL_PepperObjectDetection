#!/usr/bin/env python

import unittest
import object_detection
from object_detection import object_recognition

class TestObjectDetection(unittest.TestCase):
    def test_image_recognition(self):
        # test if outputs data
        data = {data:[]} # put the data in
        object_recognition(data)
