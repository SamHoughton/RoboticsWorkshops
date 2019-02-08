#!/usr/bin/env python

import rospy
import cv2
import numpy
from numpy import mean
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##robot
class image_converter:
    def __init__(self):
	cv2.namedWindow("HSV", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print ""

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((25, 100, 25)),
                                 numpy.array((255, 255, 255)))

	print mean(hsv_thresh)
        print '===='
	pub = rospy.Publisher('result_topic', String)
	pub.publish(str(mean(hsv_thresh)))
	cv2.imshow("HSV", hsv_thresh)

image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
