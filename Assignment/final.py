#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as maths
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
# if the service takes a long time to execute, the user might want the ability to cancel the request during execution
# or get periodic feedback about how the request is progressing.
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class findpillar:

    def __init__(self):

        # Allows for windows to be opened for the images in the program
        cv2.startWindowThread()

        # Robots Position to start will be 0, 0 on the centre spot
        self.robotLocalX = 0
        self.robotLocalX = 0

        # This is a flag point to switch when a colour is found
        self.Found = [False, False, False, False]
    
        
        self.wayPointsX = [1.5, 2, 0 , -0.5, 0 , -4.2]
        self.wayPointsY = [-4.4, 0, 0 , -1.5, 1.4 , 1]
        self.wayPointsS = [False, False, False, False, False, False]
        self.wayPointsR = [False, False, False, False, False, False]
        
        
        
        self.bridge = CvBridge()
        
        #Declaring Subscribers
        
        # One for the camera raw image to gather the input from the camera
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCallback)
        # One to gather the input from the move base 
        self.position = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.closeWaypoint)
        
        #Declares Publisher for Velocity Movement
        
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
        
        # a conversion method that is protected againsted in case it fails
        try:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # the colours for the different thresholds in two arrays to loop through
        coloursLower = [0,60,100,25]
        coloursUpper = [4,70,120,30]
        colourMask = [None]*4            
        
        #Creating the HSV Colour Bounds - Looks for Blue
        hsvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
        
        # From the workshop, declare a variable to store values
        default_low = maths.array([255, 255, 255])
        default_high = maths.array([255, 255, 205])
        
        hsvThresh = cv2.inRange(hsvImage, default_low, default_high)        
        hsv = cv2.moments(hsvThresh)

        
        coloursTogether = [hsv, hsv, hsv, hsv]
        
        for i in range(len(self.Found)):
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
        
        # Runs the image display functions
        self.imageShow(hsvThresh, hsvImage, cvImage)
        
        colourCounter = ''
        for i in range(len(self.wayPointsX)):
            if i < 4:
                # Checks the ampount of the threshold on the screen and if the colour has been found or not 
                if coloursTogether[i]['m00'] > 800000 and self.Found[i] == False:
                    hsvThresh = colourMask[i]
                    # Sets the colour counter to the correct setting
                    colourCounter = i
                    # Passes the information to beacon detection so that the robot can start moving towards it 
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
       
       #Creates height, width, depth variables
        height, width, depth = cvImage.shape
        #Creates a bound for top of shape of the thresholded colour
        topSearch = height*0.5
        #Creates a bound for top of shape of the thresholded colour
        botSearch = height*0.8
        # decides how far the robot should be before it finds the pillars
        lim = height*0.7
        
        # Cancels the current waypoint goal if a beacon is detected by the robot
        cancelGoal = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        # Actual command to cancel the goal for the robot
        cancelGoal.cancel_goal()
                
        
        # Sets the beacon to found and alerts the user that the beacon has been found
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
        
        # These are boundries so the threshold cant escape it
        hsvThresh[0:topSearch, 0:width] = 0
        hsvThresh[botSearch:height, 0:width] = 0
        
    
        # The moments store a threshold image and therefore can look for the colours
        moments = cv2.moments(hsvThresh)
        
        # Stores a moment of the threshold
        if moments['m00'] > 0:
           
           cx = int(moments['m10']/moments['m00'])
           
           cy = int(moments['m01']/moments['m00'])  
            #This creates the circle for window image
           cv2.circle(cvImage, (cx, cy), 20, (0, 0, 100), -1)
           error = cx - width/2
           # moves the robot along the X
           self.twist.linear.x = 0.5
           self.twist.angular.z = -float(error) / 100
           # Publishes the twist command to the robot
           self.cmd_vel_pub.publish(self.twist)      

        return False
         
    def moveWaypoint(self, x, y, pointNum):
        
        action = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        
        target = MoveBaseGoal()
        target.target_pose.header.frame_id = "map"
        target.target_pose.header.stamp = rospy.Time.now()
        # Specify each of the goal positions
        target.target_pose.pose.position.x = x
        target.target_pose.pose.position.y = y
        target.target_pose.pose.orientation.x = 0.0
        target.target_pose.pose.orientation.y = 0.0
        target.target_pose.pose.orientation.z = 0.0
        if pointNum == 5:
            target.target_pose.pose.orientation.z = -0.5
        target.target_pose.pose.orientation.w = 1.0
        # Send the waypoint to the navigation stack
        action.send_goal(target)
        
        self.laserS=rospy.Subscriber("/scan", LaserScan, self.laserCallback)

        for i in range(len(self.wayPointsR)):
            
            if pointNum == i:
                self.wayPointsS[i] = True
                print("Im on my way to waypoint " + str(pointNum+1))
#           
    def closeWaypoint(self, data):

        # Take data from the library and determine where the robot is in the location on the map
        self.robotposx = data.feedback.base_position.pose.position.x
        self.robotposy = data.feedback.base_position.pose.position.y
        self.robotorx = data.feedback.base_position.pose.orientation.x
        self.robotory = data.feedback.base_position.pose.orientation.y
        self.robotorz = data.feedback.base_position.pose.orientation.z
        error = 0.50     

        # The srttings to insure that the waypoints close and the data is saved
        for i in range(len(self.wayPointsR)):
            if self.wayPointsS[i] == True and self.wayPointsR[i] == False:
                if self.robotposx >= (self.wayPointsX[i] - error) and self.robotposx <= (self.wayPointsX[i] + error) and self.robotposy >= (self.wayPointsY[i]- error) and self.robotposy <= (self.wayPointsY[i] + error):
                    self.wayPointsR[i] = True

        
cv2.startWindowThread()
rospy.init_node('findpillar', anonymous=True)
findpillar()
rospy.spin()
cv2.destroyAllWindows()