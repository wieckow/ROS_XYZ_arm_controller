#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi, cos, sin

L1 = 22.0
L2 = 22.0
L3 = 79.0
L4 = 70.0
L5 = 127.0

scaling = 62
rad_res = 0.00563
deg_res = 0.323

rospy.init_node('forward_kinematics')

def enc2rad(enc):
    rad = []
    rad.append((enc[0] - 512)*rad_res)
    rad.append(pi/2.0 - ((enc[1] - 474)*rad_res))
    rad.append(((enc[2] - 429)*rad_res) + pi/2.0)
    rad.append(((enc[3] - 567)*rad_res) + pi)
    rad.append((enc[4] - 512)*rad_res)
    rad.append(enc[5])
    return rad

def enc2deg(enc):
    deg = []
    deg.append((enc[0] - 512)*deg_res)
    deg.append(90 - (enc[1] - 474)*deg_res)
    deg.append((enc[2] - 429)*deg_res + 90)
    deg.append((enc[3] - 567)*deg_res + 180)
    deg.append((enc[4] - 512)*deg_res)
    deg.append(enc[5])
    return deg

def joint2cart(rad):
    x1 = -L2
    z1 = +L1

    x2 = x1 + L3*cos(rad[1])
    z2 = z1 + L3*sin(rad[1])

    x3 = x2 + L4 * cos(rad[2] + rad[1] - pi)
    z3 = z2 + L4 * sin(rad[2] + rad[1] - pi)

    x4 = x3 + L5 * cos(rad[1] + rad[2] + rad[3])
    z4 = z3 + L5 * sin(rad[1] + rad[2] + rad[3])

    roll = (rad[4]) % (2*pi)
    pitch = (rad[1] + rad[2] + rad[3]) % (2*pi)
    grip = rad[5]

    return [x4*cos(rad[0]), x4*sin(rad[0]), z4, roll, pitch, grip]

# def setpointCallback(data):
#     enc = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
#     rad = enc2rad(enc)
#     cart = joint2cart(rad)
#     msg = Twist()
#     msg.linear.x = cart[0]
#     msg.linear.y = cart[1]
#     msg.linear.z = cart[2]
#     msg.angular.x = cart[3]
#     msg.angular.y = cart[4]
#     msg.angular.z = cart[5]
#     setpoint_pub.publish(msg)

def currposCallback(data):
    enc = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    rad = enc2rad(enc)
    cart = joint2cart(rad)
    msg = Twist()
    msg.linear.x = cart[0]
    msg.linear.y = cart[1]
    msg.linear.z = cart[2]
    msg.angular.x = cart[3]
    msg.angular.y = cart[4]
    msg.angular.z = cart[5]
    currpos_pub.publish(msg)

def setpointCallback(data):
    enc = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    rad = enc2rad(enc)
    cart = joint2cart(rad)
    msg = Twist()
    msg.linear.x = cart[0]
    msg.linear.y = cart[1]
    msg.linear.z = cart[2]
    msg.angular.x = cart[3]
    msg.angular.y = cart[4]
    msg.angular.z = cart[5]
    setpoint_pub.publish(msg)

setpoint_sub = rospy.Subscriber("setpoint_joint", Twist, setpointCallback)
currpos_sub = rospy.Subscriber("currpos_joint", Twist, currposCallback)

setpoint_pub = rospy.Publisher("setpoint_cart", Twist, queue_size=1)
currpos_pub = rospy.Publisher("currpos_cart", Twist, queue_size=1)

rospy.spin()