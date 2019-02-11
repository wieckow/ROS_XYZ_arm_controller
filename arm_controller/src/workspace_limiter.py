#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

rospy.init_node('workspace_limiter')

X_MIN = 35
X_MAX = 150

Y_MIN = -100
Y_MAX = 100

Z_MIN = 0
Z_MAX = 150

YAW_MIN = -pi/4.0
YAW_MAX = pi/4.0

ROLL_MIN = -pi/2.0
ROLL_MAX = pi/2.0

clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

def setpointCallback(data):
    # data.linear.x = clamp(data.linear.x, X_MIN, X_MAX)
    # data.linear.y = clamp(data.linear.y, Y_MIN, Y_MAX)
    # data.linear.z = clamp(data.linear.z, Z_MIN, Z_MAX)
    # data.angular.x = clamp(data.angular.x, ROLL_MIN, ROLL_MAX)
    # data.angular.y = clamp(data.angular.y, YAW_MIN, YAW_MAX)
    setpoint_pub.publish(data)

setpoint_sub = rospy.Subscriber("setpoint_cart", Twist, setpointCallback)

setpoint_pub = rospy.Publisher("setpoint_cart_lim", Twist, queue_size=1)

rospy.spin()