#!/usr/bin/env python

from __future__ import print_function

PKG = 'unittest_python'
NAME = 'camera_unittest'

import yaml
import os
import sys, time
import unittest
from unittest_python.msg import Position_32, Position_64
import rospy, rospkg
import rostest

class YAMLParaser():

    def __init__(self):
        self.position = list()

    def parse_file(self):
        folder_path = rospkg.RosPack().get_path('unittest_python')+"/config"
        for f in os.listdir(folder_path):
            child_folder_path = os.path.join(folder_path, f)
            file_path = child_folder_path + "/info.yaml"
            document = open(file_path, 'r')
            yaml_parser = yaml.safe_load(document)
            self.position.append([float(yaml_parser['boxes']['box_0']['position']['x']),float(yaml_parser['boxes']['box_0']['position']['y']),float(yaml_parser['boxes']['box_0']['position']['z'])])
        return self.position

class TestCamera(unittest.TestCase):

    def __init__(self, *args):
        super(TestCamera, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        self.success_32 = False
        self.success_64 = False
        self.msg_64 = Position_64()
        self.msg_32 = Position_32()
        
    def callback_64(self, data):
        self.msg_64 = data
        if data.x == 0.250360752237 and data.y == -0.0315066584742 and data.z == 1.29800144243:
            self.success_64 = True

    def callback_32(self, data):
        self.msg_32 = data
        if data:
            self.success_32 = True

    def test_publisher_32(self):
        sub_32 = rospy.Subscriber("camera_position_32", Position_32, self.callback_32)
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not self.success_32 and time.time() < timeout_t:
            time.sleep(0.1)
        print("Checking if any value is recieved")
        self.assert_(self.success_32)

    def test_publisher_64(self):
        sub_64 = rospy.Subscriber("camera_position_64", Position_64, self.callback_64)
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not self.success_64 and time.time() < timeout_t:
            time.sleep(0.1)
        print("Checking if the value recieved is correct")
        self.assert_(self.success_64)
    
    def test_value_file(self):
        self.skipTest("another method for skipping")
        yaml = YAMLParaser()
        positions = yaml.parse_file()
        tests = [[0.2485468459350091, -0.03188925889539131, 1.151412462746655],[0.2524309428293764, -0.03335709823870366, 1.443761817467065],[0.2503607522374318, -0.03150665847423637, 1.298001442426517],[0.2480109412637022, -0.03164193199740516, 1.004698523768672]]
        for i, position in enumerate(tests):
            self.assertEquals(position[0],positions[i][0],"")
            self.assertEquals(position[1],positions[i][1])
            self.assertEquals(position[2],positions[i][2])

        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestCamera, sys.argv)