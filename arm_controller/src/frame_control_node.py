#! /usr/bin/env python

import rospy
import numpy as np
# import arm_controller.frame_params as fdef
import std_msgs.msg
from arm_controller.msg import arm_command_frame as CmdFrame
from arm_controller.msg import arm_return_frame as ReturnFrame

def returnCallback(rx_data):
    if(len(rx_data.data) == 0):
        rospy.logwarn("Empty return frame received")
    else:
        msg = ReturnFrame()
        msg.type = np.uint8(rx_data.data[0])
        msg.data_field = []
        if(len(rx_data.data) > 1):
            msg.data_field = rx_data.data[1:]
        tmp_data = []
        for i in range(len(msg.data_field)):
            tmp_data.append(msg.data_field[i] if msg.data_field[i] >= 0 else msg.data_field[i] + 256)
        msg.data_field = tmp_data
        return_pub.publish(msg)

def commandCallback(rx_data):
    msg = std_msgs.msg.ByteMultiArray(data=[0 for i in range(len(rx_data.data_field)+1)])
    msg.data[0] = rx_data.cmd
    for i in range(len(rx_data.data_field)):
        value = (ord(rx_data.data_field[i]) & 0xFF)
        value = value - 256 if value > 127 else value
        msg.data[i+1] = value
    msg.layout.data_offset = len(rx_data.data_field)
    command_pub.publish(msg)


rospy.init_node('Frame_controller')

command_pub = rospy.Publisher("arm_command", std_msgs.msg.ByteMultiArray, queue_size=1)
return_pub = rospy.Publisher("arm_return_frame",  ReturnFrame, queue_size=1)

return_sub = rospy.Subscriber("arm_return", std_msgs.msg.ByteMultiArray, returnCallback)
command_sub = rospy.Subscriber("arm_command_frame", CmdFrame, commandCallback)

rospy.loginfo('Frame_controller node initialized')

rospy.spin()