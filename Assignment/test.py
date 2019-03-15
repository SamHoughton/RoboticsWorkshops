#!/usr/bin/env python
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
        cv.startWindowThread()
        
        # Current position of the robot in its local location on the robotic
        # a.k.a where the robot starts once the simulation is loaded
        self.localx = 0
        self.localy = 0
        self.i = 1
        
        # Sets variables for the colours of the pillars, T/F based on when the 
        # colours are discovered
        self.redFound = False
        self.yellowFound = False
        self.blueFound = False
        self.greenFound = False
        
        # Storage for the waypoints using na array
        
        self.wayPoints = []
        waypoint1 = tuple([1.5, -4.4, False])
        waypoint2 = tuple([1.5, 0, False])
        waypoint3 = tuple([2, 0, False])
        waypoint4 = tuple([-0.5, -.4, False])
        waypoint5 = tuple([1.5, -4.4, False])
        waypoint6 = tuple([1.5, -4.4, False])
        
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
        
        # Exception handling to ensure data is passed through if not it erroirs
        try:
            # Convert the data from the sensors to make them openCV
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, error:
            # catches the error when converting the image   
            print error
            
        # storing the image and converts the image
        hsvImage = cv.cvtColor(cvImage, cv.COLOR_BGR2HSV)
        
        # gets the general shape for the image of the camera
        height, width, depth = cvImage.shape
        
        # back up variable       
        hsvThresh = cv.inRange(hsvImage,
                                     numpy.array((255, 255, 255)),
                                     numpy.array((255, 255, 205)))

        # stores the pixels to be used by different colours in case colours dont work
        hsvPixels = cv.moments(hsvThresh)
        r = hsvPixels
        y = hsvPixels
        b = hsvPixels
        g = hsvPixels
        
        
        # sets the statements to check to see the different colours using the thresholds

        redColour = cv.inRange(hsvImage,                                     
                               numpy.array((0, 200, 30)),
                               numpy.array((5, 255, 150)))  
        r = cv.moments(redColour)
        # Same for yellow
        yellowColour = cv.inRange(hsvImage,
                                     numpy.array((30, 200, 30)),
                                     numpy.array((40, 255, 200)))
        y = cv.moments(yellowColour)
        # Same for blue
        blueColour = cv.inRange(hsvImage,
                                     numpy.array((90, 200, 30)),
                                     numpy.array((120, 255, 240)))
        b = cv.moments(blueColour)
        # Same for green
        greenColour = cv.inRange(hsvImage,
                                     numpy.array((60, 200, 30)),
                                     numpy.array((70, 255, 200)))
        g = cv.moments(greenColour)
        
        # code for the red circle on a pillar if found
                
        hsvThresh, hsvContours, hierachy = cv.findContours(hsvThresh.copy(),
                                                  cv.RETR_TREE,
                                                  cv.CHAIN_APPROX_SIMPLE)
        for c in hsvContours:
            a = cv.contourArea(c)
            if a > 100.0:
                cv.drawContours(cvImage, c, -1, (255, 0, 0))
                
        # End condition for the code and therefore completing goals set
                
        if self.redFound == True and self.yellowFound == True and self.blueFound == True and self.greenFound == True:
            ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            ac.cancel_goal()            
            rospy.loginfo("All objects found!")
            rospy.signal_shutdown("Found all objects")
        
        # Checks to see which colour the robot sees within a distance of 50000 and if the colour has not
        # been found yet
        
        if r['m00'] > 50000 and self.redColour == False:
            # Sets the colour in the thresh 
            hsvThresh = redColour
            # Pass the data so the robot moves towards the beacon 
            self.redFound = self.BeaconDetection(cvImage, hsvThresh)
        elif y['m00'] > 50000 and self.yellowColour == False:
            # Sets the colour in the thresh 
            hsvThresh = yellowColour
            # Pass the data so the robot moves towards the beacon 
            self.yellow = self.BeaconDetection(cvImage, hsvThresh)
        elif b['m00'] > 50000 and self.blueColour == False:
            # Sets the colour in the thresh 
            hsvThresh = blueColour
            # Pass the data so the robot moves towards the beacon 
            self.blue = self.BeaconDetection(cvImage, hsvThresh)
        elif g['m00'] > 50000 and self.greenColour == False:
            # Sets the colour in the thresh 
            hsvThresh = greenColour
            # Pass the data so the robot moves towards the beacon 
            self.green = self.BeaconDetection(cvImage, hsvThresh)
        # if it cannot see any of the colours, it will move to the waypoints through a map
        else: 
            self.WaypointMove(self.wayPoints[self.i][1],self.wayPoints[self.i][2])
            self.i = self.i + 1
        # Display current coloured thresholded image as well as the  raw image for inspection    
        cv.imshow("Image window", hsvThresh)
        cv.waitKey(1)
    
    def BeaconDetection(self, cvImage, hsvThresh): 
        
        height, width, depth = cvImage.shape
        
        hsvTest = hsvThresh
        
        search_top = height*0.5
        search_bot = height * 0.79
        limit = height * 0.80
        
        
        # Tells the robot to move and use only one goal at the time
        ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        ac.cancel_goal()
        
        # Checks to see if the colour is in front of the robot
        if hsvTest[limit:height, 0:width].any():
            # If the colour is there returns the code
            rospy.loginfo("Found object!")
            return True
        
        hsvThresh[0:search_top, 0:width] = 0
        hsvThresh[search_bot:height, 0:width] = 0
        CurrentPosition = cv.moments(hsvThresh)
        
        # Check that object is present in thresholded image    
        if CurrentPosition['m00'] > 0:
            
            # Determine the distance of the object from the centre of the screen
            # in order to correct the path of the robot if it's off course
            cx = int(CurrentPosition['m10']/CurrentPosition['m00'])
            cy = int(CurrentPosition['m01']/CurrentPosition['m00'])
            cv.circle(cvImage, (cx, cy), 20, (0,0,255), -1)
            # Build the twist message to send to the robot telling it to move from workshop
            if (cy >= 100): #crude way to stop
                twist_msg = Twist();
                error = cx - width/2
                # Telss the robot to move
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = -float(error) / 100
                self.vel_pub.publish(twist_msg)          
        # return false if the image of the pillar could not be found 
        return False
    
     # Defining function to move the robot between waypoints around the map     
    def WaypointMove(self, x, y, goalNum):
        
        # Again creates a place for commands to be sent
       ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

       # Wait for the action client to be available from workshops 
       while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting for the move_base action server to come up")
           
     
       # Begin setting up the goal message to send to the stack by using a map
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id = "map"
       goal.target_pose.header.stamp = rospy.Time.now()
       
       # Specify each of the goal positions
       goal.target_pose.pose.position.x = x
       goal.target_pose.pose.position.y = y
       goal.target_pose.pose.orientation.x = 0.0
       goal.target_pose.pose.orientation.y = 0.0
       goal.target_pose.pose.orientation.z = 0.0
       if goalNum == 6:
           goal.target_pose.pose.orientation.z = -0.5
       goal.target_pose.pose.orientation.w = 1.0
       
       # Send the waypoint to the navigation stack
       ac.send_goal(goal)
       
       if goalNum == 1:
           self.Waypoints[1][3] = True
           print goalNum
       elif goalNum == 2:
           self.Waypoints[2][3] = True
           print goalNum
       elif goalNum == 3:
           self.Waypoints[3][3] = True
           print goalNum
       elif goalNum == 4:
           self.Waypoints[4][3] = True
           print goalNum
       elif goalNum == 5:
           self.Waypoints[5][3] = True
           print goalNum
       elif goalNum == 6:
           self.Waypoints[6][3] = True
           print goalNum