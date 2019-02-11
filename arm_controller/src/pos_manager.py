#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import arm_controller.frame_params as conf# pylint: disable=E0611, E0401

from arm_controller.msg import arm_command_frame as CmdFrame
from arm_controller.msg import arm_return_frame as ReturnFrame

def returnCallback(data):
    if(data.type == conf.RET_POS_ALL):
        pos = []
        for i in range(6):
            pos.append(ord(data.data_field[2*i])*256 + ord(data.data_field[2*i + 1]))
        msg = Twist()
        msg.linear.x = pos[0]
        msg.linear.y = pos[1]
        msg.linear.z = pos[2]
        msg.angular.x = pos[3]
        msg.angular.y = pos[4]
        msg.angular.z = pos[5]
        print msg
        pos_pub.publish(msg)


def pos_callback(data):
    msg = CmdFrame()
    msg.cmd = conf.CMD_AUTO_SETPOINT
    pos = [int(data.linear.x), int(data.linear.y), int(data.linear.z), int(data.angular.x), int(data.angular.y), int(data.angular.z)]
    vals = []
    for i in range(6):
        vals.append((pos[i] & 0xFF00) >> 8)
        vals.append(pos[i] & 0xFF)
    msg.data_field = vals
    cmd_pub.publish(msg)

rospy.init_node('pos_translator')

return_sub= rospy.Subscriber("arm_return_frame", ReturnFrame, returnCallback)
pos_sub = rospy.Subscriber("setpoint_joint_lim", Twist, pos_callback)

cmd_pub = rospy.Publisher("arm_command_frame", CmdFrame, queue_size=1)
pos_pub = rospy.Publisher("currpos_joint", Twist, queue_size=1)

rospy.spin()