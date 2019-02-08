#!/usr/bin/env python

import rospy
import cv2
import numpy
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
       ## self.image_sub = rospy.Subscriber("/camera/image_raw/",
                                          ##Image, self.callback)
                                          
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)


    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((0, 0, 0)),
                                 numpy.array((25, 158, 125)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((0, 0, 0)),
                                 numpy.array((75, 84, 62)))

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])

        hsv_thresh, bgr_contours, hierachy = cv2.findContours(
            bgr_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        hsv_thresh, hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        print '===='
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        
        
        print mean(hsv_thresh)
        print '===='
        p = rospy.Publisher('result_topic', String)
        p.publish(str(mean(hsv_thresh)))
        cv2.imshow("HSV", hsv_thresh)


image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
