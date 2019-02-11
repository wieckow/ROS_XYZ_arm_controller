#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi, cos, sin, acos, asin, atan, sqrt, atan2

L1 = 22.0
L2 = 22.0
L3 = 79.0
L4 = 70.0
L5 = 127.0

rad_res = 0.00563

rospy.init_node('inverse_kinematics')

def rad2enc(rad_):
    enc = []
    rad = rad_[:]
    for i in range(len(rad)-1):
        rad[i] = rad[i]% (2.0 * pi)
        rad[i] = rad[i] if rad[i] < pi else rad[i] - 2*pi
    enc.append(int((rad[0]/rad_res) + 512))
    enc.append(int(((pi/2.0-rad[1])/rad_res)+474))
    enc.append(int(((rad[2] - pi/2.0)/rad_res) + 429))
    enc.append(int(((rad[3] - pi)/rad_res) + 567))
    enc.append(int((rad[4])/rad_res + 512))
    enc.append(int(rad[5]))
    return enc

def cart2joint(rad):
    x = rad[0]
    y = rad[1]
    z = rad[2]
    pitch = rad[4]
    roll = rad[3]
    grip = rad[5]

    theta = [0.0] * 6
    theta[5] = grip
    theta[4] = roll% (2.0 * pi)
    theta[0] = atan2(float(y),float(x))% (2.0 * pi)

    m = sqrt(y**2 + x**2) + L2
    n = z - L1
    x3 = m - L2 - L5*cos(pitch)
    y3 = n + L1 - L5*sin(pitch)

    A = sqrt( (x3+L2)**2 + (y3-L1)**2 )
    theta[2] = acos( (L3**2 + L4**2 - A**2)/(2*L3*L4) )
    beta = atan2((y3-L1),(x3+L2))
    gamma = acos( (L3**2 + A**2 - L4**2)/(2*L3*A) )
    theta[1] = (beta+gamma)% (2.0 * pi)
    theta[3] = (pitch - theta[1] - theta[2] + 2.0 * pi) % (2.0 * pi)
    return theta

def setpointCallback(data):
    rad = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    try:
        joint = cart2joint(rad)
        joint = rad2enc(joint)
        msg = Twist()
        msg.linear.x = joint[0]
        msg.linear.y = joint[1]
        msg.linear.z = joint[2]
        msg.angular.x = joint[3]
        msg.angular.y = joint[4]
        msg.angular.z = joint[5]
        setpoint_pub.publish(msg)
    except ValueError:
        rospy.logwarn("Position unreachable")
        
setpoint_sub = rospy.Subscriber("setpoint_cart_lim", Twist, setpointCallback)

setpoint_pub = rospy.Publisher("setpoint_joint_lim", Twist, queue_size=1)

rospy.spin()