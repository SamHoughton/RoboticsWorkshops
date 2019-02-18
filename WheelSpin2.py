#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

wheel_radius = .1
robot_radius = 1


class Kinematics:
	def __init__(self):
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.input_sub = rospy.Subscriber('/wheel_vel_left', Float32, self.forward_kinematics)
		self.twist = Twist()

	def forward_kinematics(self, w_l):
		c_l = 1.0 * w_l.data
		c_r = 1.0 * 0.0
		v = (c_l + c_r) / 2
		a = (c_r - c_l) / (2 * 1.0)
		print(v,a)
		self.twist.linear.x = v
		self.twist.angular.z = a
		self.vel_pub.publish(self.twist)

rospy.init_node('kinematics')
kinematics = Kinematics()
rospy.spin()

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
