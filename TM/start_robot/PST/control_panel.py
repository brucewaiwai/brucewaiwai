#!/usr/bin/env python
#-*- coding: utf-8 -*-
# Date: 2020/12/7
# Version: 1.0
print('===== Version: 1.0 =====')

import subprocess
import sys
#from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import os
import rospy

import Tkinter as tk
import math

window = tk.Tk()
window.title('TM')
window.geometry('800x700')
window.configure(background='white')



def Actuator_on():
    cmd = String()
    cmd.data = 'up'
    pub_Actuator_cmd.publish(cmd)
    print(cmd)

def Actuator_off():
    cmd = String()
    cmd.data = 'down'
    pub_Actuator_cmd.publish(cmd)
    print(cmd)

def Servo_on():
    cmd = Bool()
    cmd.data = True
    pub_grabR_cmd.publish(cmd)
    pub_grabL_cmd.publish(cmd)

def Servo_off():
    cmd = Bool()
    cmd.data = False
    pub_grabR_cmd.publish(cmd)
    pub_grabL_cmd.publish(cmd)
##########################
# def reset_pos():
#     cmd = String()
#     cmd.data = 'reset_position'
#     pub3.publish(cmd)
#     print(cmd)

def startmove():
    x1 = float(height_entry.get())
    y1 = float(weight_entry.get())
    z1 = float(angle_entry.get())
    cmd = String()
    cmd.data = "StartMove:"+str(x1)+","+str(y1)+","+str(z1)
    pub_server_cmd.publish(cmd)
    print(cmd)

def startcam():
    subprocess.call("gnome-terminal -x bash -ic ;'rqt_image_view'", shell=True)
    
def startkey():
    subprocess.call("gnome-terminal -x bash -ic 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py'", shell=True)

def change_map():
    subprocess.call("gnome-terminal -x bash -ic 'rosnode kill /loc_map /nav_map'", shell=True)

def reset_position():
    cmd = String()
    cmd.data = 'Reset_position'
    pub_server_cmd.publish(cmd)

def on_press(event):
    print('hi')
    cmd = Bool()
    cmd.data = True
    pub_stop.publish(cmd)
def on_release(event):
    print('bye')
    cmd = Bool()
    cmd.data = False
    pub_stop.publish(cmd)

def reboot():
    os.system('reboot')
    
def exit():
    sys.exit(0)
    
def start():
    rospy.init_node('control_panel')
    global pub_Actuator_cmd
    pub_Actuator_cmd = rospy.Publisher('/base_control/Actuator_cmd', String, queue_size=10)
    global pub_server_cmd
    pub_server_cmd = rospy.Publisher('/tm/server_cmd', String, queue_size=10)
    global pub_grabR_cmd
    pub_grabR_cmd = rospy.Publisher('/RightServo', Bool, queue_size = 5)
    global pub_grabL_cmd
    pub_grabL_cmd = rospy.Publisher('/LeftServo', Bool, queue_size = 5)
    global pub_stop
    pub_stop = rospy.Publisher('/tm/stop', Bool, queue_size = 5)

    




if __name__ == "__main__":

    start()

    header_label = tk.Label(window, text='TMrobot')
    header_label.pack()
#---------------------------------startmove
    xyz_frame = tk.Frame(window)
    tk.Label(xyz_frame, text="StartMove", width = 10, height = 3, font=("blue", 10)).grid(row = 0, column = 0)

    height_label = tk.Label(xyz_frame, text='x')
    height_entry = tk.Entry(xyz_frame, width = 5)

    weight_label = tk.Label(xyz_frame, text='y')
    weight_entry = tk.Entry(xyz_frame, width = 5)

    angle_label = tk.Label(xyz_frame, text='Angle')
    angle_entry = tk.Entry(xyz_frame, width = 5)

    startmove = tk.Button(xyz_frame, text='Go', command=startmove)

    xyz_frame.place(x = 20, y = 60)

    height_label.grid(row = 1, column = 0)
    height_entry.grid(row = 1, column = 1)
    weight_label.grid(row = 2, column = 0)
    weight_entry.grid(row = 2, column = 1)
    angle_label.grid(row = 3, column = 0)
    angle_entry.grid(row = 3, column = 1)

    startmove.grid(row = 4, column = 1)




#------------------------------------function
    test = tk.Frame(window)
	
    actuator_on = tk.Button(test, text='actuator on', width = 15, height = 4, command=Actuator_on)
    servo_on = tk.Button(test, text='servo on', width = 15, height = 4, command=Servo_on)
    # airfon = tk.Button(test, text='airf on', width = 15, height = 4, command=airfon)
    # uvon = tk.Button(test, text='uv on', width = 15, height = 4, command=uvon)
    # coveron = tk.Button(test, text='cover on', width = 15, height = 4, command=coveron)
    # up = tk.Button(test, text='up', width = 15, height = 4, command=up)

    actuator_off = tk.Button(test, text='actuator off', width = 15, height = 4, command=Actuator_off)
    servo_off = tk.Button(test, text='servo off', width = 15, height = 4, command=Servo_off)
    # airfoff = tk.Button(test, text='airf off', width = 15, height = 4, command=airfoff)
    # uvoff = tk.Button(test, text='uv off', width = 15, height = 4, command=uvoff)
    # coveroff = tk.Button(test, text='cover off', width = 15, height = 4, command=coveroff)
    # down = tk.Button(test, text='down', width = 15, height = 4, command=down)

	
    test.place(x = 330, y = 150)
    actuator_on.grid(row = 0, column = 0)
    servo_on.grid(row = 1, column = 0)
    # airfon.grid(row = 2, column = 0)
    # uvon.grid(row = 3, column = 0)
    # coveron.grid(row = 4, column = 0)
    # up.grid(row = 5, column = 0)

    actuator_off.grid(row = 0, column = 1)
    servo_off.grid(row = 1, column = 1)
    # airfoff.grid(row = 2, column = 1)
    # uvoff.grid(row = 3, column = 1)
    # coveroff.grid(row = 4, column = 1)
    # down.grid(row = 5, column = 1)



#-------------------------------------start camera
    tk.Button(window, text='Start Camera', width = 10, height = 1, command=startcam).place(x = 340, y = 60)
#-------------------------------------start keyboard
    tk.Button(window, text='Start Keyboard', width = 10, height = 1, command=startkey).place(x = 480, y = 60)
#-------------------------------------reset position
    tk.Button(window, text='Reset Position', width = 10, height = 1, command=reset_position).place(x = 340, y = 90)
#-------------------------------------change map
    tk.Button(window, text='change map', width = 10, height = 1, command=change_map).place(x = 480, y = 90)
#-------------------------------------reboot
    tk.Button(window, text='Reboot', width = 10, height = 1, command=reboot).place(x = 620, y = 90)
#-------------------------------------cheat
    stop = tk.Button(window, text='test', width = 40, height = 7)
    stop.place(x = 200, y = 320)
    stop.bind("<ButtonPress>", on_press)
    # stop.bind("<ButtonRelease>", on_release)

    start = tk.Button(window, text='test2', width = 40, height = 7)
    start.place(x = 200, y = 440)
    start.bind("<ButtonPress>", on_release)
    # start.bind("<ButtonRelease>", on_release)



    exit_button = tk.Button(window, text = "Quit", width = 30, height = 4, command = exit)
    exit_button.pack(side=tk.BOTTOM)
	
    window.mainloop()

