#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class findpillar:

    def __init__(self):
        
        #command to start displaying to visual windows
        cv2.startWindowThread()
        
        # Current position of the robot in its local location
        self.x = 0
        self.y = 0
        
        # Sets variables for the colours of the pillars
        self.red = False
        self.yellow = False
        self.blue = False
        self.green = False