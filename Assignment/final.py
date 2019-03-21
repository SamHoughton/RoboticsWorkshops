#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as maths
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
# Imports the action library for the users
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class findpillar:

    def __init__(self):

        # Allows for windows to be opened for the images in the program for program to display raw and segmented images
        cv2.startWindowThread()

        # Robots Position to start will be 0, 0 on the centre spot of the arena of pillars
        self.robotLocalX = 0
        self.robotLocalX = 0

        # This is a list to switch parts to true when a colour is found
        self.Found = [False, False, False, False]
    
        # These are the waypoints of the robot to go visit around the map, one for the x and one for the y
        self.wayPointsX = [1.5, 2, 0 , -0.5, 0 , -4.2]
        self.wayPointsY = [-4.4, 0, 0 , -1.5, 1.4 , 1]
        # These are waypoints sent and waypoints recieved by the robot and will also be switched when found
        self.wayPointsS = [False, False, False, False, False, False]
        # Waypoints Recieved
        self.wayPointsR = [False, False, False, False, False, False]       
        # provides an interface between the robot images and open CV images
        self.bridge = CvBridge()
        
        # One for the camera raw image to gather the input from the camera
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCallback)
        # A simple position subscriber to gather information about robot movement
        self.position = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.closeWaypoint)
        
        #Declares the publisher for the movement of the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)          
        # Declares the twist command for the robot
        self.twist = Twist()
    
    def laserCallback (self, msg):

        # The laser callback to turn the robot if it comes within a certain distance of an object
        if msg.ranges[-10] < 1:
            rate = rospy.Rate(10)
            # twist once the robot is in range of the application
            self.twist.angular.z = -0.1
            # Send the command by publishing the mesage of self.twist
            self.cmd_vel_pub.publish(self.twist) 
            rate.sleep()
        # if the range is the other way in terms of angles
        elif msg.ranges[10] < 1:
            rate = rospy.Rate(10)
            # Reverse the twist
            self.twist.angular.z = 0.1
            # Publish the twist
            self.cmd_vel_pub.publish(self.twist) 
            rate.sleep()
            
        
          
    def imageCallback(self, data):
        
        # a conversion method that is protected againsted in case it fails, will print the error if failure occurs
        try:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, error:
            print error
        
        # the colours for the different thresholds in two arrays to loop through
        # Each four of the colour boundries are stored here and will needed to be looped through
        coloursLower = [0,60,100,25]
        coloursUpper = [4,70,120,30]
        # Stores an empty colour mask for the different colours
        colourMask = [None]*4            
        
        # Converts the image into the correct colour type to be displayed
        hsvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
        
        # From the workshop, declare a variable to store values
        default_low = maths.array([255, 255, 255])
        # Creates a back up one in case the colours fail
        default_high = maths.array([255, 255, 205])
        
        # Makes the threshold for the colours using the arrays
        hsvThresh = cv2.inRange(hsvImage, default_low, default_high)   
        # Stores the threshold in a moment to ensure it isnt lost
        hsv = cv2.moments(hsvThresh)

        # Groups the colours together 
        coloursTogether = [hsv, hsv, hsv, hsv]
        
        # For i in the length of the colours found, in this case 4
        for i in range(len(self.Found)):
            # Loop through the different colour masks and ensure that each colour is passed in
            colourMask[i] = cv2.inRange(hsvImage, maths.array([coloursLower[i], 100, 100]), maths.array([coloursUpper[i], 255, 255])) 
            if self.Found[i] == False:
                coloursTogether[i] = cv2.moments(colourMask[i])
                
                # draws a nice red cicle on the thing its aiming at                 
                if coloursTogether[i]['m00'] > 0: #If object has an area
                      cx = int(coloursTogether[i]['m10']/coloursTogether[i]['m00'])
                      cy = int(coloursTogether[i]['m01']/coloursTogether[i]['m00'])
                      cv2.circle(cvImage, (cx, cy), 20, (255,0,0), -1)
            
        # Adds the colours into one threshold - From the workshops
        hsvThresh = colourMask[0]+colourMask[1]+colourMask[2]+colourMask[3]

        self.imageShow(hsvThresh, hsvImage, cvImage)
        
        colourCounter = ''
        for i in range(len(self.wayPointsX)):
            if i < 4:
                # Checks the ampount of the threshold on the screen and if the colour has been found or not 
                if coloursTogether[i]['m00'] > 800000 and self.Found[i] == False:
                    hsvThresh = colourMask[i]
                    colourCounter = i
                    self.Found[i] = self.BeaconDetect(cvImage, hsvThresh, colourCounter)
                    break
            else:
                for i in range(len(self.wayPointsR)):
                    
                    if i == 0:
                        if self.wayPointsR[i] == False and self.wayPointsS[i] == False:
                            self.moveWaypoint(self.wayPointsX[i], self.wayPointsY[i], i)
                    else:
                        if self.wayPointsR[i] == False and self.wayPointsR[i-1] == True and self.wayPointsS[i] == False:
                            self.moveWaypoint(self.wayPointsX[i], self.wayPointsY[i], i)
    
        if self.Found[0] == True and self.Found[1] == True and self.Found[2] == True and self.Found[3] == True:
            print('All objects have been found, congrats.')
            exit()
   
   # This is to show the images in a window for the program        
    def imageShow(self, hsvThresh, hsvImage, cvImage):

        # again form the workshop to display and show the image
        _, hsvContours, hierachy = cv2.findContours(
            hsvThresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
                
        #Updates the window
        cv2.imshow("Binary Image Window", hsvThresh)
        cv2.waitKey(1) 
        # Shows the original camera feed from the robot
        cv2.imshow("Image Raw", cvImage)  
        cv2.waitKey(1)

    def BeaconDetect(self, cvImage, hsvThresh, colourCounter):
        
        height, width, depth = cvImage.shape
        #Top of the shape of the thresholded colour for example red bottom
        topSearch = height*0.5
        #Top of the shape of the thresholded colour for example red top
        botSearch = height*0.8
        # decides how far the robot should be before it finds the pillars
        lim = height*0.7
        
        # Cancels the current waypoint goal if a beacon is detected by the robot
        cancelGoal = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        # Actual command to cancel the goal for the robot
        cancelGoal.cancel_goal()
                
        
        # Returns true for the beacon being found and tells which one has been found
        if hsvThresh[lim:height, 0:width].any():
            if colourCounter == 0:
                print("Found Red!")
            elif colourCounter == 1:
                print("Found Green!")
            elif colourCounter == 2:
                print("Found Blue!")
            elif colourCounter == 3:
                print("Found Yellow!")
            return True        
        
        # These threshold boundries keep it from escaping
        hsvThresh[0:topSearch, 0:width] = 0
        hsvThresh[botSearch:height, 0:width] = 0
        
    
        # The moments store a threshold image and therefore can look for the colours
        moments = cv2.moments(hsvThresh)
        
        # Stores a moment of the threshold
        if moments['m00'] > 0:
           
           cx = int(moments['m10']/moments['m00'])    
           cy = int(moments['m01']/moments['m00'])  
           cv2.circle(cvImage, (cx, cy), 20, (0, 0, 100), -1)
           error = cx - width/2
           # moves the robot along the X at the speed of 0.5
           self.twist.linear.x = 0.5
           self.twist.angular.z = -float(error) / 100
           self.cmd_vel_pub.publish(self.twist)      

        return False
         
    def moveWaypoint(self, x, y, pointNum):
        
        action = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        
        target = MoveBaseGoal()
        target.target_pose.header.frame_id = "map"
        target.target_pose.header.stamp = rospy.Time.now()
        # Specify each of the goal positions from the waypoint coordinates
        target.target_pose.pose.position.x = x
        target.target_pose.pose.position.y = y
        target.target_pose.pose.orientation.x = 0.0
        target.target_pose.pose.orientation.y = 0.0
        target.target_pose.pose.orientation.z = 0.0
        if pointNum == 5:
            target.target_pose.pose.orientation.z = -0.5
        target.target_pose.pose.orientation.w = 1.0
        # Send the waypoint to the navigation stack to the top
        action.send_goal(target)
        
        self.laserS=rospy.Subscriber("/scan", LaserScan, self.laserCallback)

        for i in range(len(self.wayPointsR)):
            
            if pointNum == i:
                self.wayPointsS[i] = True
                print("Im on my way to waypoint " + str(pointNum+1))
#   definition for the close waypoint function        
    def closeWaypoint(self, data):

        # Take data from the library and determine where the robot is in the location on the map
        self.robotposx = data.feedback.base_position.pose.position.x
        self.robotposy = data.feedback.base_position.pose.position.y
        self.robotorx = data.feedback.base_position.pose.orientation.x
        self.robotory = data.feedback.base_position.pose.orientation.y
        self.robotorz = data.feedback.base_position.pose.orientation.z
        error = 0.50     

        # The settings to ensure that the waypoints are set to close and that the next wyapoint can be moved to by the robot
        for i in range(len(self.wayPointsR)):
            if self.wayPointsS[i] == True and self.wayPointsR[i] == False:
                if self.robotposx >= (self.wayPointsX[i] - error) and self.robotposx <= (self.wayPointsX[i] + error) and self.robotposy >= (self.wayPointsY[i]- error) and self.robotposy <= (self.wayPointsY[i] + error):
                    self.wayPointsR[i] = True

        
cv2.startWindowThread()
rospy.init_node('findpillar', anonymous=True)
findpillar()
rospy.spin()
cv2.destroyAllWindows()
