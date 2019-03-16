#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Importing necessary libraries
import rospy
import cv2
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
#  if the service takes a long time to execute, the user might want the ability to cancel the request during execution
# or get periodic feedback about how the request is progressing.
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class findpillar:

    def __init__(self):

        cv2.startWindowThread()

        self.localx = 0
        self.localy = 0

        self.Found = [False, False, False, False]
    
        
        self.wayPointsX = [1.5, 2, 0 , -0.5, 0 , 1.5]
        self.wayPointsY = [-4.4, 0, 0 , -1.5, 1.4 , -4.5]
        self.wayPointsS = [False, False, False, False, False, False]
        self.wayPointsR = [False, False, False, False, False, False]
        
        self.laser=LaserScan()
        self.laserS=rospy.Subscriber("/scan", LaserScan, self.laserCallback)

        
        self.bridge = CvBridge()
        
        #Declaring Subscribers
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCallback)
        self.position = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.closeWaypoint)
        
        #Declares Publisher for Velocity Movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)  

        rospy.loginfo("Starting node")
                
        self.twist = Twist()
    
    def laserCallback (self, msg):
        self.laser= msg      
    
    def imageCallback(self, data):
        
        try:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        
        hl = [0,60,100,25]
        hu = [4,70,120,30]
        colourMask = [None]*4            
        
        #Creating the HSV Colour Bounds - Looks for Blue
        hsv_img = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
        
        default_low = numpy.array([255, 255, 255])
        default_high = numpy.array([255, 255, 205])
        
        hsvThresh = cv2.inRange(hsv_img, default_low, default_high)        
        hsv = cv2.moments(hsvThresh)

        coloursTogether = [hsv, hsv, hsv, hsv]
        
        for i in range(len(self.Found)):
            colourMask[i] = cv2.inRange(hsv_img, numpy.array([hl[i], 100, 100]), numpy.array([hu[i], 255, 255])) 
            if self.Found[i] == False:
                coloursTogether[i] = cv2.moments(colourMask[i])
            
        # Adds the colours into one threshold - From the workshops
        hsvThresh = colourMask[0]+colourMask[1]+colourMask[2]+colourMask[3]
        
        # Runs the image display functions
        self.imageShow(hsvThresh, hsv_img, cvImage)
        
        colourCounter = ''
        for i in range(len(self.wayPointsX)):
            if i < 4:
                # The distance to the colours and if the colour selectected has not been found
                if coloursTogether[i]['m00'] > 500000 and self.Found[i] == False:
                    hsvThresh = colourMask[i]
                    # Sets the colour counter to the correct setting
                    colourCounter = i
                    # Passes the information to moveObject
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
    
    def imageShow(self, hsvThresh, hsv_img, cvImage):

        _, hsvContours, hierachy = cv2.findContours(
            hsvThresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
         
        #Draws the new image in a window displaying in Binary just the blue objects
        for c in hsvContours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(hsv_img, c, -1, (0, 0, 255))
                
        #Updates the window
        cv2.imshow("Segmented Image Window", hsvThresh)
        cv2.waitKey(1)     
        cv2.imshow("Image Raw", cvImage)  
        cv2.waitKey(1)

    def BeaconDetect(self, cvImage, hsvThresh, colourCounter):
       
       #Creates height, width, depth variables
        h, w, d = cvImage.shape
        #Creates a bound for top of shape
        search_top = h*0.5
        #Creates a bound for bottom of shape
        search_bot = h*0.8
        
        lim = h*0.85
        
        action = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        action.cancel_goal()
                
        
        # If any of the thresholded object is directly in front of the robot
        # return true to change the status of the beacon to found
        if hsvThresh[lim:h, 0:w].any():
            if colourCounter == 0:
                print("Found Red!")
            elif colourCounter == 1:
                print("Found Green!")
            elif colourCounter == 2:
                print("Found Blue!")
            elif colourCounter == 3:
                print("Found Yellow!")
            return True        
        
        #I think this decides if the colours found is within the bounds of the shape found
        hsvThresh[0:search_top, 0:w] = 0
        hsvThresh[search_bot:h, 0:w] = 0
        
    
        #I'm guessing creates some moment thing
        M = cv2.moments(hsvThresh)
        
        #This is what decides if the turtle should move toward the object or not
        if M['m00'] > 0:
           
           cx = int(M['m10']/M['m00'])
           cy = int(M['m01']/M['m00'])
            
            #This creates the circle for window image
           cv2.circle(cvImage, (cx, cy), 20, (0, 0, 100), -1)
           err = cx - w/2
           self.twist.linear.x = 0.5
           self.twist.angular.z = -float(err) / 100

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


        # Change the corresponding waypoint status to "sent" to stop the program
        # from flooding the stack with the same message when another callback is called
        for i in range(len(self.wayPointsR)):
            
            if pointNum == i:
                self.wayPointsS[i] = True
                print("Heading to Waypoint " + str(pointNum+1))
                print(str(x) + ' & ' + str(y))
#           
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
                if self.robotposx >= (self.wayPointsX[i] - error) and self.robotposx <= (self.wayPointsX[i] + error) and self.robotposy >= (self.wayPointsY[i]- error) and self.robotposy <= (self.wayPointsY[i] + error):
                    self.wayPointsR[i] = True

        
cv2.startWindowThread()
rospy.init_node('findpillar', anonymous=True)
findpillar()
#rospy.init_node("Follower")
#Follower()
rospy.spin()
cv2.destroyAllWindows()