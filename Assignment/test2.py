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
        
        # Sets variables for the colours of the pillars, T/F based on when the 
        # colours are discovered
        self.redFound = False
        self.yellowFound = False
        self.blueFound = False
        self.greenFound = False
        
        # Storage for the waypoints using na array
        
        self.wayPointsX = [1.5, 2, 0 , -0.5, 0 , 1.5]
        self.wayPointsY = [-4.4, 0, 0 , -1.5, 1.4 , -4.5]
        self.wayPointsS = [False, False, False, False, False, False]
        self.wayPointsR = [False, False, False, False, False, False]
        

        # Converts the ROS robots images to OpenCV
        self.bridge = CvBridge()
        
        # subscribes to the camera feed and stores the feed
        self.imageSub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.imageCallback)
        
        # Gets the robots current position and subscribes
        self.robotPosition = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.closeWaypoint)  

        # Gets the velocity of the robot and subscribes to it
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)                          
    
    def imageCallback(self, data):
        
        # This gives a window a name for the image input        
        cv.namedWindow("Image Window", 1)
        
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
                
        hsvThresh = blueColour + redColour + yellowColour + greenColour
        
        hsvThresh, hsvContours, hierachy = cv.findContours(hsvThresh.copy(),
                                                  cv.RETR_TREE,
                                                  cv.CHAIN_APPROX_SIMPLE)
        for c in hsvContours:
            a = cv.contourArea(c)
            if a > 100.0:
                cv.drawContours(cvImage, c, -1, (255, 0, 0))
           
         
        # Display current coloured thresholded image as well as the  raw image for inspection    
        cv.imshow("Image window", hsvThresh)
        cv.waitKey(1)             
             
        # End condition for the code and therefore completing goals set
                
        if self.redFound == True and self.yellowFound == True and self.blueFound == True and self.greenFound == True:
            ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            ac.cancel_goal()            
            rospy.loginfo("All objects found!")
            rospy.signal_shutdown("Found all objects")
        
        # Checks to see which colour the robot sees within a distance of 50000 and if the colour has not
        # been found yet
        
        if r['m00'] > 650000 and self.redFound == False:
            # Sets the colour in the thresh
            print('nearly found red')
            hsvThresh = redColour
            # Pass the data so the robot moves towards the beacon 
            self.redFound = self.BeaconDetection(cvImage, hsvThresh)
            print(str(self.redFound) + " is Red")
        elif y['m00'] > 650000 and self.yellowFound == False:
            # Sets the colour in the thresh 
            hsvThresh = yellowColour
            # Pass the data so the robot moves towards the beacon 
            self.yellowFound = self.BeaconDetection(cvImage, hsvThresh)
        elif b['m00'] > 650000 and self.blueFound == False:
            # Sets the colour in the thresh 
            hsvThresh = blueColour
            # Pass the data so the robot moves towards the beacon 
            self.blueFound = self.BeaconDetection(cvImage, hsvThresh)
        elif g['m00'] > 650000 and self.greenFound == False:
            # Sets the colour in the thresh 
            hsvThresh = greenColour
            # Pass the data so the robot moves towards the beacon 
            self.greenFound = self.BeaconDetection(cvImage, hsvThresh)
        # if it cannot see any of the colours, it will move to the waypoints through a map
        else: 
            for i in range(len(self.wayPointsX)):
                #print('in')
                self.WaypointMove(self.wayPointsX[i], self.wayPointsY[i], i)
                    
       
    
    def BeaconDetection(self, cvImage, hsvThresh): 
        
        height, width, depth = cvImage.shape
        
        #hsvTest = hsvThresh
        
        search_top = height*0.5
        search_bot = height * 0.79
        limit = height * 0.85
        
        
        # Tells the robot to move and use only one goal at the time
        ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        ac.cancel_goal()
        
        # Checks to see if the colour is in front of the robot
        if hsvThresh[limit:height, 0:width].any():
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
           
       print('waypoint in')
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
       if goalNum == 5:
           goal.target_pose.pose.orientation.z = -0.5
       goal.target_pose.pose.orientation.w = 1.0
       
       # Send the waypoint to the navigation stack
       ac.send_goal(goal)
       
       for i in range(len(self.wayPointsX)):
         if goalNum == i:
            self.wayPointsS[i] = True

    
    def closeWaypoint(self, data):

        # Take data from move_base/feedback to find the current position of the
        # robot
        self.robotposx = data.feedback.base_position.pose.position.x
        self.robotposy = data.feedback.base_position.pose.position.y
        self.robotorx = data.feedback.base_position.pose.orientation.x
        self.robotory = data.feedback.base_position.pose.orientation.y
        self.robotorz = data.feedback.base_position.pose.orientation.z
        error = 0.50     
        
                
        # Check whether the robot's current position is within the error region 
        # specified, if so set the waypoint's status to found and increment 
        # the number of waypoints found.

        for i in range(len(self.wayPointsR)):
            if self.wayPointsS[i] == True and self.wayPointsR[i] == False:
               if self.robotposx >= (self.wayPointsX[i] - error) and self.robotposx <= (self.wayPointsX[i] + error) and self.robotposy >= (self.wayPointsY[i] - error) and self.robotposy <= (self.wayPointsY[i] + error):
                   self.wayPointsR[i] = True

           
if __name__ == '__main__':
    # Initialise a node of the find_objects class
    rospy.init_node("find_objects", anonymous=True)
    cvHelp = findpillar()
    
    rospy.spin()
    cv.destroyAllWindows()