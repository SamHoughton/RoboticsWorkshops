#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Control:
	def __init__(self):
		self.input_pub = rospy.Publisher('/wheel_vel_left', Float32, queue_size=10)
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


wheel_radius = .1
robot_radius = 1


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
