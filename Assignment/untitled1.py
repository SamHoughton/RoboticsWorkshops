#!/us-4.4r/bin/env python
# -*- coding: utf-8 -*-


import rospy
import cv2 as cv
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class findpillar:

    def __init__(self):
        
        #command to start displaying windows for the camera and the threshold
        cv2.startWindowThread()
        
        # Current position of the robot in its local location on the robotic
        # a.k.a where the robot starts once the simulation is loaded
        self.localx = 0
        self.localy = 0
        
        # Sets variables for the colours of the pillars, T/F based on when the 
        # colours are discovered
        self.red = False
        self.yellow = False
        self.blue = False
        self.green = False
        
        # Storage for the waypoints using na array
        
        self.wayPoints = []
        waypoint1 = tuple([1.5, -4.4])
        waypoint2 = tuple([1.5, 0])
        waypoint3 = tuple([2, 0])
        waypoint4 = tuple([-0.5, -.4])
        waypoint5 = tuple([1.5, -4.4])
        waypoint6 = tuple([1.5, -4.4])
        self.wayPoints.append(waypoint1, waypoint2, waypoint3, waypoint4, waypoint5, waypoint6)
        
        # Converts the ROS robots images to OpenCV
        self.bridge = CvBridge()
        
        # subscribes to the camera feed and stores the feed
        self.imageSub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        
        # Gets the robots current position and subscribes
        self.robotPosition = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.closeWaypoint)  

        # Gets the velocity of the robot and subscribes to it
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)                          
    
    def imageCallback(self, data):
        
        # This gives a window a name for the image input        
        cv.namedWidnow("Image Window", 1)
        # Exception handling to ensure data is passed through
        try:
            # Convert the data from the sensors to make them openCV
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, error:
            # catches the error when converting the image   
            print error
            
        # storing the image
        hsvImage = cv.cvtColor(cvImage, cv.COLOR_BGR2HSV)
        
        # gets the general shape for the image of the camera
        height, width, depth = cvImage.shape
        
        #        
        hsv_thresh = cv.inRange(hsv_img,
                                     numpy.array((255, 255, 255)),
                                     numpy.array((255, 255, 205)))

        # stores the pixels 
        hsv_pixels = cv.moments(hsv_thresh)
        r = hsv_pixels
        y = hsv_pixels
        b = hsv_pixels
        g = hsv_pixels