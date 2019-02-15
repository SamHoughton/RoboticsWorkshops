#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Control:
	def __init__(self):
		self.input_pub = rospy.Publisher('/turtlebot_1/wheel_vel_left', Float32, queue_size=10)
		vel = input("")		
		self.publish_velocity(vel)
		
	
	def publish_velocity(self,vel):
		while not rospy.is_shutdown():
			self.input_pub.publish(vel)
			r.sleep()

rospy.init_node('control')
r = rospy.Rate(10)
control = Control()
rospy.spin()
