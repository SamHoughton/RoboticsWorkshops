#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError

class Follower:
  def __init__(self):
      
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()

  def depth_callback(self, ros_image):
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        # Convert the depth image using the default passthrough encoding
        depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
    except CvBridgeError, e:
        print e

    # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
    depth_array = numpy.array(depth_image, dtype=numpy.float32)
                
    # Normalize the depth image to fall between 0 (black) and 1 (white)
    cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

    # Process the depth image
    depth_display_image = self.process_depth_image(depth_array)
    
    # Display the result
    cv2.imshow("depth", depth_display_image)

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Colour range from Week 3
    mask = cv2.inRange(hsv,
                        numpy.array((40, 100, 100)),
                numpy.array((80, 255, 255)))

    # Image dimensions for calculations
    h, w, d = image.shape

    # Find objects in image
    M = cv2.moments(mask)
       
    

    
    if M['m00'] > 100000: #If object has an area
      cx = int(M['m10']/M['m00']) 
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
     
     # Navigation
      err = cx - w/2
      print(cx, cy) # Object location (x,y)
      if (cx >= 300): # A way to stop the robot.
      	self.twist.linear.x = 0
      	self.twist.angular.z = 0
      	self.cmd_vel_pub.publish(self.twist)
      else:
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.2
        self.cmd_vel_pub.publish(self.twist)
          
       
    # Display what robot is moving to
    cv2.imshow("image", image)
    cv2.imshow("target", mask)
    cv2.waitKey(3)

  def process_depth_image(self, frame):
    # Just return the raw image for this demo
    return frame

rospy.init_node('follower')
follower = Follower()
rospy.spin()
