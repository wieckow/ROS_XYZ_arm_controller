#! /usr/bin/env python

import rospy
import arm_controller.frame_params as conf# pylint: disable=E0611, E0401
from arm_controller.msg import arm_command_frame as CmdFrame
from arm_controller.msg import arm_return_frame as ReturnFrame

rospy.init_node('Arm_monitor')

def commandCallback(cmd_frame):
    command =  cmd_frame.cmd
    if command in conf.COMMAND_LIST:
        conf.vis_cmd[command].display(cmd_frame.data_field)

def returnCallback(ret_frame):
    ret_type =  ret_frame.type
    if ret_type in conf.RETURN_FRAMES:
        conf.vis_ret[ret_type].display(ret_frame.data_field)

command_sub = rospy.Subscriber("arm_command_frame", CmdFrame, commandCallback)
command_sub = rospy.Subscriber("arm_return_frame", ReturnFrame, returnCallback)

rospy.loginfo('Arm_monitor node initialized')

rospy.spin()