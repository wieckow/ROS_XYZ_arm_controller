#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import arm_controller.frame_params as conf# pylint: disable=E0611, E0401
from arm_controller.msg import arm_command_frame as CmdFrame
from arm_controller.msg import arm_return_frame as ReturnFrame
from math import pi


import Tkinter as tk

rospy.init_node('arm_interface')

root = tk.Tk()

dt = 500
logdt = 200

class App:
    def __init__(self, master):
                
        # self.ret_sub = rospy.Subscriber("arm_ros_return", ReturnFrame, self.returnCallback)
        self.command_pub = rospy.Publisher("arm_command_frame",  CmdFrame, queue_size=1)
        self.joint_setpos = rospy.Publisher("setpoint_joint",  Twist, queue_size=1)
        self.cart_sub = rospy.Subscriber("currpos_cart", Twist, self.cartCallback)
        self.joint_sub = rospy.Subscriber("currpos_joint", Twist, self.jointCallback)
        self.cart_setpos = rospy.Publisher("setpoint_cart",  Twist, queue_size=1)
        self.curr_pos_joint = []
        self.curr_pos_cart = []
        self.logging = False
        self.logcnt = 0
        self.logfile = None
        self.logtime = None
        for i in range(6):
            self.curr_pos_cart.append(tk.IntVar())
            self.curr_pos_joint.append(tk.IntVar())
        self.pos_labels_joint = []
        self.pos_sliders_joint = []
        self.pos_labels_cart = []
        self.pos_sliders_cart = []
        self.regulation = True
        self.safe_pos = [512, 413, 460, 367, 796, 515]
        self.safe_pos_cart = [116, 0, 54, 91, -36, 511]
        self.inverse = False
        mins = [80, 322, 302, 262, 73, 510]
        maxs = [970, 700, 740, 721, 950, 943]
        cart_mins = [60, -100, 0, -120, -45, 510]
        cart_maxs = [160, 100, 150, 120, 45, 943]
        self.jpos_label = tk.Label(text="Joint pos", width=15, font='bold', fg = 'green')
        self.jpos_label.grid(row=0, column=0, columnspan=3, sticky=tk.N+tk.W+tk.S+tk.E, ipady=10)
        self.cpos_label = tk.Label(text="Cartesian pos", width=15, font='bold', fg = 'red')
        self.cpos_label.grid(row=0, column=3, columnspan=3, sticky=tk.N+tk.W+tk.S+tk.E, ipady=10)
        tk.Label(text='X', width=15, height=2).grid(row=1, column=3)
        tk.Label(text='Y', width=15, height=2).grid(row=2, column=3)
        tk.Label(text='Z', width=15, height=2).grid(row=3, column=3)
        tk.Label(text='Roll', width=15, height=2).grid(row=4, column=3)
        tk.Label(text='Pitch', width=15, height=2).grid(row=5, column=3)
        tk.Label(text='Grip', width=15, height=2).grid(row=6, column=3)
        for i in range(6):
            self.pos_labels_joint.append(tk.Label(master))
            tk.Label(text='Servo ' + str(i+1), width=15, height=2).grid(row=i+1, column=0)
            self.pos_labels_joint[i].grid(row = i+1, column = 1)
            self.pos_labels_joint[i].configure(text=str(self.curr_pos_joint[i].get()), width=15, height=2)
            self.pos_sliders_joint.append(tk.Scale(master, from_=mins[i], to=maxs[i], orient=tk.HORIZONTAL))
            self.pos_sliders_joint[i].set(self.safe_pos[i])
            self.pos_sliders_joint[i].grid(row=i+1, column = 2)
            self.pos_labels_cart.append(tk.Label(master))
            self.pos_labels_cart[i].grid(row = i+1, column = 4)
            self.pos_labels_cart[i].configure(text=str(self.curr_pos_joint[i].get()), width=15, height=2)
            self.pos_sliders_cart.append(tk.Scale(master, from_=cart_mins[i], to=cart_maxs[i], orient=tk.HORIZONTAL))
            self.pos_sliders_cart[i].set(self.safe_pos_cart[i])
            self.pos_sliders_cart[i].grid(row=i+1, column = 5)
            self.pos_sliders_cart[i].configure(state=tk.DISABLED)
        self.safepos_button = tk.Button(master, text="Safe Position", command=self.safePosition)
        self.safepos_button.grid(row=7, column = 0)
        self.monitor_button = tk.Button(master, text="Monitor position", command=self.toggleMonitor)
        self.monitor_button.grid(row=7, column = 1)
        self.regulation_button = tk.Button(master, text="Toggle regulation", command=self.toggleRegulation)
        self.regulation_button.grid(row=7, column = 2)
        self.kinematics_button = tk.Button(master, text="Forward/inverse kin", command=self.toggleKin)
        self.kinematics_button.grid(row=7, column = 3)
        self.logging_button = tk.Button(master, text="start/stop logging", command=self.toggleLog)
        self.logging_button.grid(row=7, column = 4)


        root.after(dt, self.update)
        root.after(logdt, self.logcb)

    def rosinit(self):
        msg = CmdFrame()
        msg.cmd = conf.CMD_AUTO_MONITOR
        self.command_pub.publish(msg)
        msg = CmdFrame()
        msg.cmd = conf.CMD_AUTO_ON
        self.command_pub.publish(msg)

    def update(self):
        if self.inverse:
            cart_msg = Twist()
            cart_msg.linear.x = self.pos_sliders_cart[0].get()
            cart_msg.linear.y = self.pos_sliders_cart[1].get()
            cart_msg.linear.z = self.pos_sliders_cart[2].get()
            cart_msg.angular.x = self.pos_sliders_cart[3].get()*pi/180.0
            cart_msg.angular.y = self.pos_sliders_cart[4].get()*pi/180.0
            cart_msg.angular.z = self.pos_sliders_cart[5].get()
            cart_msg.angular.x = cart_msg.angular.x if cart_msg.angular.x >= 0 else cart_msg.angular.x + 2.0*pi            
            self.cart_setpos.publish(cart_msg)
        else:
            jt_msg = Twist()
            jt_msg.linear.x = self.pos_sliders_joint[0].get()
            jt_msg.linear.y = self.pos_sliders_joint[1].get()
            jt_msg.linear.z = self.pos_sliders_joint[2].get()
            jt_msg.angular.x = self.pos_sliders_joint[3].get()
            jt_msg.angular.y = self.pos_sliders_joint[4].get()
            jt_msg.angular.z = self.pos_sliders_joint[5].get()
            self.joint_setpos.publish(jt_msg)
        root.after(dt, self.update)

    def logcb(self):
        tm = 0
        if self.logging:
            tm = rospy.Time.now() - self.logtime
            tm = int(tm.nsecs / 1000000) + 1000 * tm.secs
            self.logfile.write(str(tm) + ',')
            if self.inverse:
                cart_msg = Twist()
                cart_msg.linear.x = self.pos_sliders_cart[0].get()
                cart_msg.linear.y = self.pos_sliders_cart[1].get()
                cart_msg.linear.z = self.pos_sliders_cart[2].get()
                cart_msg.angular.x = self.pos_sliders_cart[3].get()*pi/180.0
                cart_msg.angular.y = self.pos_sliders_cart[4].get()*pi/180.0
                cart_msg.angular.z = self.pos_sliders_cart[5].get()
                cart_msg.angular.x = cart_msg.angular.x if cart_msg.angular.x >= 0 else cart_msg.angular.x + 2.0*pi       
                self.logfile.write(str(cart_msg.linear.x) + ',')
                self.logfile.write(str(cart_msg.linear.y) + ',')
                self.logfile.write(str(cart_msg.linear.z) + ',')
                self.logfile.write(str(cart_msg.angular.x) + ',')
                self.logfile.write(str(cart_msg.angular.y) + ',')
                self.logfile.write(str(cart_msg.angular.z) + ',')
                for i in self.curr_pos_cart[:-1]:
                    self.logfile.write(str(i.get()) + ',')
                self.logfile.write(str(self.curr_pos_cart[-1].get()) + '\n')
            else:
                jt_msg = Twist()
                jt_msg.linear.x = self.pos_sliders_joint[0].get()
                jt_msg.linear.y = self.pos_sliders_joint[1].get()
                jt_msg.linear.z = self.pos_sliders_joint[2].get()
                jt_msg.angular.x = self.pos_sliders_joint[3].get()
                jt_msg.angular.y = self.pos_sliders_joint[4].get()
                jt_msg.angular.z = self.pos_sliders_joint[5].get()
                self.logfile.write(str(jt_msg.linear.x) + ',')
                self.logfile.write(str(jt_msg.linear.y) + ',')
                self.logfile.write(str(jt_msg.linear.z) + ',')
                self.logfile.write(str(jt_msg.angular.x) + ',')
                self.logfile.write(str(jt_msg.angular.y) + ',')
                self.logfile.write(str(jt_msg.angular.z) + ',')
                for i in self.curr_pos_joint[:-1]:
                    self.logfile.write(str(i.get()) + ',')
                self.logfile.write(str(self.curr_pos_joint[-1].get()) + '\n')
        root.after(logdt, self.logcb)

    def toggleMonitor(self):
        msg = CmdFrame()
        msg.cmd = conf.CMD_AUTO_MONITOR
        self.command_pub.publish(msg)

    def toggleLog(self):
        if self.logging:
            self.logging = False
            self.logfile.close()
            self.logfile = None
        else:
            self.logcnt = self.logcnt+1
            name = "logs/Log_"+str(self.logcnt)+'.csv'
            self.logfile = file(name, 'w')
            self.logtime = rospy.Time.now()
            self.logging = True

    def safePosition(self):
        msg = CmdFrame()
        msg.cmd = conf.CMD_SAFE_POS
        self.command_pub.publish(msg)
        for i in range(6):
            self.pos_sliders_joint[i].set(self.safe_pos[i])

    def toggleRegulation(self):
        msg = CmdFrame()
        msg.cmd = conf.CMD_AUTO_OFF if self.regulation else conf.CMD_AUTO_ON
        self.regulation = not self.regulation
        self.command_pub.publish(msg)

    def toggleKin(self):
        self.inverse = not self.inverse
        if self.inverse:
            for i in range(6):
                self.pos_sliders_joint[i].configure(state = tk.DISABLED)
                self.pos_sliders_cart[i].configure(state = tk.NORMAL)
                self.pos_sliders_cart[i].set(self.curr_pos_cart[i].get())
            self.jpos_label.configure(fg='red')
            self.cpos_label.configure(fg='green')
        else:
            for i in range(6):
                self.pos_sliders_joint[i].configure(state = tk.NORMAL)
                self.pos_sliders_cart[i].configure(state = tk.DISABLED)
                self.pos_sliders_cart[i].set(self.curr_pos_cart[i].get())
            self.jpos_label.configure(fg='green')
            self.cpos_label.configure(fg='red')


    # def returnCallback(self, data):
    #     if data.type == conf.RET_POS_ALL:
    #         for i in range(6):
    #             self.curr_pos_joint[i].set((ord(data.data_field[2*i]) * 256) + ord(data.data_field[2*i + 1]))
    #             self.pos_labels_joint[i].configure(text=str(self.curr_pos_joint[i].get()))    
    def cartCallback(self, data):
        pos = [data.linear.x, data.linear.y, data.linear.z, data.angular.x * 180/pi, data.angular.y * 180/pi, data.angular.z]
        pos[4] = pos[4] -360 if pos[4] > 180 else pos[4]
        for i in range(6):
            self.curr_pos_cart[i].set(int(pos[i]))
            self.pos_labels_cart[i].configure(text=str(int(pos[i])))  

    def jointCallback(self, data):
        pos = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
        for i in range(6):
            self.curr_pos_joint[i].set(int(pos[i]))
            self.pos_labels_joint[i].configure(text=str(int(pos[i])))  


a = App(root)
root.mainloop()
