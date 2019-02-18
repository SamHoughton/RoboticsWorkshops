# -*- coding: utf-8 -*-
"""
Spyder Editor
This is a temporary script file.
"""
import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

wheel_radius = .1
robot_radius = 1

class DirectControl:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/wheel_vel_left', data,
                                          self.control_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,
                                           queue_size=1)
        self.twist = Twist()
        
    def control_callback(self, data):
        

# computing the forward kinematics for a differential drive
        def forward_kinematics(w_l, w_r):
            c_l = wheel_radius * w_l
            c_r = wheel_radius * w_r
            v = (c_l + c_r) / 2
            a = (c_r - c_l) / (2 * robot_radius)
            return (v, a)
        
        
        # computing the inverse kinematics for a differential drive
        def inverse_kinematics(v, a):
            c_l = v - (robot_radius * a)
            c_r = v + (robot_radius * a)
            w_l = c_l / wheel_radius
            w_r = c_r / wheel_radius
            return (w_l, w_r)
        
        
        # inverse kinematics from a Twist message (This is what a ROS robot has to do)
        def inverse_kinematics_from_twist(t):
            return inverse_kinematics(t.linear.x, t.angular.z)
        
        if __name__ == "__main__":
        
            (w_l, w_r) = inverse_kinematics(0.0, 1.0)
            print "w_l = %f,\tw_r = %f" % (w_l, w_r)
        
            (v, a) = forward_kinematics(w_l, w_r)
            print "v = %f,\ta = %f" % (v, a)
        
            from geometry_msgs.msg import Twist
            t = Twist()
        
            t.linear.x = 0.3
            t.angular.z = 0.8
        
            (w_l, w_r) = inverse_kinematics_from_twist(t)
            print "w_l = %f,\tw_r = %f" % (w_l, w_r)
