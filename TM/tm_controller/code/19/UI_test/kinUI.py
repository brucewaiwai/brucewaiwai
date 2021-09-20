#!/usr/bin/env python
# Date: 2021/08/27
# Version: 1.0

import time
import random
import threading
import os
import rospy
import subprocess
import math

import Tkinter as tk
from Tkinter import *
import tkFileDialog as filedialog
import ttk
from PIL import Image
from PIL import ImageTk
import imageio

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tm_controller.msg import battery

#================main window===============
root = Tk()
root.title('TM-Test')
# root.geometry("1024x600")
# root.minsize(500, 300)
root.configure(background='white')
root.attributes('-fullscreen', True)

background_label = Label(root,
                         #highlightbackground="black",highlightthickness=1, 
                         bg = 'white')
background_label.place(x=-1, y=-175, relwidth=1.0, relheight=1.5)

#================img,video,icon & character=======================
# PATH = "/home/ken/catkin_ws/src/kinrobot_test"
PATH = "/home/tmrobot/catkin_ws/src/TM/tm_controller/UI_test"
emergency_img = PhotoImage(file = PATH + "/img/kinTest_img/emergency-icon_resize2.png")
grabbingArm_img = PhotoImage(file = PATH + "/img/kinTest_img/grabbing_arm_resize.png")
gamepad_img = PhotoImage(file = PATH + "/img/kinTest_img/gamepad2_resize.png")
gamepadCross_img = PhotoImage(file = PATH + "/img/kinTest_img/gamepad2_cross_resize.png")
# openArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/open_arrow_resize.png")
# closeArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/close_arrow_resize.png")
openArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/grabbing_arm_open2_resize.png")
closeArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/grabbing_arm_close2_resize.png")

# upArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/up_arrow_resize.png")
# downArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/down_arrow_resize.png")
upArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/load_resize.png")
downArrow_img = PhotoImage(file = PATH + "/img/kinTest_img/unload_resize.png")
autoParking_img = PhotoImage(file = PATH + "/img/kinTest_img/auto_parking_resize.png")
joystick_on_img = PhotoImage(file = PATH + "/img/kinTest_img/joystick_on3_resize.png")
joystick_off_img = PhotoImage(file = PATH + "/img/kinTest_img/joystick_off3_resize.png")

carParking_img = PhotoImage(file = PATH + "/img/kinTest_img/car_park_resize.png")
exitIcon_img = PhotoImage(file = PATH + "/img/kinTest_img/exit_resize.png")
battery_video = imageio.get_reader(PATH + "/img/battery_edited2.mp4")
white_img = PhotoImage(file = PATH + "/img/kinTest_img/white.png")

face = []
for i in range(1,14):
    face_img = Image.open(PATH + "/img/face/Face_%i.png"%i)
    face_img = face_img.resize((700,700), Image.ANTIALIAS)
    face_img = ImageTk.PhotoImage(face_img)
    face.append(face_img)
    # face.append(tk.PhotoImage(file = "/home/ken/Desktop/img/face/Face_%i.png"%i))

#================ROS Topic========================================
#-------------Publish Topic----------------
gripperTopic = '/base_control/Servo'
actuatorTopic = '/base_control/Actuator_cmd'
autoChargingTopic = '/start_charging'
joystickTopic = '/turn_on_joy_controller'
#-------------Subscribe Topic--------------
emergencyTopic = "/EM_button"
batteryTopic = "/state/battery/info"
odomDistTopic = "/odom_dist"
odomTopic = "/odom"

#================frame============================================
frame = Frame(root)
frame.pack()



upper_frame = Frame(root,bg = "white",
                    #highlightbackground="black",highlightthickness=1
                    )
upper_frame.pack( side = TOP , fill = X)

right_frame = Frame(root,
                    #highlightbackground="black",highlightthickness=1
                    )
right_frame.pack( side = RIGHT )

upperRight_frame = Frame(upper_frame,bg = "white", 
                         highlightbackground= '#3A7482',highlightthickness=2
                         #,width = 150, height = 50
                         )
upperRight_frame.pack( side = RIGHT)
# upperRight_frame.place(x = 874, y = 0)



left_frame = Frame(root,
                   #highlightbackground="black",highlightthickness=1,
                   bg = 'white',
                   width = 100, height = 400)
# left_frame.place(x = 0, y = 150)
left_frame.place(x = 0, y = 10) 


#-------------hide frame if background is clicked----------
frame_toggle = 0

def toggle_frame():
    global frame_toggle
    if frame_toggle == 0: 
        upper_frame.pack_forget()
        upperRight_frame.pack_forget()
        left_frame.place_forget()
        frame_toggle = 1
    elif frame_toggle == 1:
        upper_frame.pack( side = TOP , fill = X)
        upperRight_frame.pack( side = RIGHT )
        left_frame.place(x = 0, y = 10)
        frame_toggle = 0
        
background_label.bind("<Button-1>", lambda x: toggle_frame())
root.bind('<Escape>',quit)

#===============callback function============

def eye_stream(label):
    while 1:
        try:
            img = random.randint(1,13)
            t = random.randint(1,5)
            label.config(image=face[img])
            time.sleep(t)
        except Exception as e:
            print(e)


def battery_stream(label):
    while 1:
        try:
            battery_level = rospy.wait_for_message(batteryTopic, battery, timeout=5).battery_level
            # battery_level = 23.0
            frame = int((1000 - battery_level * 10 + 5))
            # print(frame)
            img = battery_video.get_data(frame)
            img = img[15:,:]  #trim image
            frame_image = ImageTk.PhotoImage(Image.fromarray(img))
            # print("================================")
            # print(img[1,:])
            # print(len(img[:,1]))
            label.config(image=frame_image)
            label.image = frame_image
            time.sleep(0.1)
            
        except Exception as e:
            print(e)


# ==========================define label=============
is_joy_turn_on = True

def roscore_callback():
    subprocess.call("gnome-terminal -x bash -ic 'cd ~/catkin_ws;roscore'", shell=True)


def gripperClose_callback(event):
    print("Gripper Closing")
    cmd = String()
    cmd.data = 'close'
    pub_servo_cmd.publish(cmd)
    rospy.loginfo('servo on')

def gripperOpen_callback(event):
    print("Gripper Opening")
    cmd = String()
    cmd.data = 'open'
    pub_servo_cmd.publish(cmd)
    rospy.loginfo('servo off')
    
def gripperStop_callback(event):
    print("Gripper Stop")
    
def actuatorUp_callback(event):
    print("Actuator UP")
    cmd = String()
    cmd.data = 'up'
    pub_actuator_cmd.publish(cmd)
    rospy.loginfo(str(cmd))
    
def actuatorDown_callback(event):
    print("Actuator DOWN")
    cmd = String()
    cmd.data = 'down'
    pub_actuator_cmd.publish(cmd)
    rospy.loginfo(str(cmd))
    
def actuatorStop_callback(event):
    print("Actuator Stop")
    
def joystick_callback():
    global is_joy_turn_on
    is_joy_turn_on = not(is_joy_turn_on)
    print(is_joy_turn_on)
    if is_joy_turn_on == True:
        gamepad_icon.config(image = gamepad_img)
        joystick_button.config(image = joystick_on_img)
    else:
        gamepad_icon.config(image = gamepadCross_img)
        joystick_button.config(image = joystick_off_img)
    print("Joy Controller")
    cmd = Bool()
    cmd = is_joy_turn_on
    pub_joystick_activate.publish(cmd)

def odomDistCallback(data):
    ############################Odom Distance##########################
    global odomDist_label
    odomDist = float(data.data)
    if odomDist < 1000.0:
        odomDist_label.config(text = str(round(odomDist, 2)) + 'm')
    elif odomDist >= 1000.0:
        odomDist_label.config(text = str(round((odomDist/1000.0),2)) + 'km')
    else:
        odomDist_label.config(text = '0.00m')

def velCallback(data):
    global velValue_label
    velResult = math.sqrt(data.twist.twist.linear.x ** 2 + data.twist.twist.linear.y **2)
    velValue_label.config(text = str(round(velResult, 2)) + ' m/s')

def autoParking_callback():
    print("Auto Parking")
    cmd = String()
    cmd = 'start'
    pub_charge_cmd.publish(cmd)

def emergency_callback():
    print("EM")

#================button & label====================

borderColor = '#3A7482' #set the color of the border
button_size = 80

emergency_icon = Label(upperRight_frame,
                        #highlightbackground="black",highlightthickness=1,bd = 2,
                        #image = emergency_img,
                        image = white_img,
                        height= 40, width = 40, 
                        bg = 'white'
                        )
carParking_icon = Label(upperRight_frame, 
                        #highlightbackground="black",highlightthickness=1,bd = 2 ,
                        # image = carParking_img,
                        image = white_img,
                        height= 40, width = 40,
                        bg = 'white'
                        )
battery_label = Label(upperRight_frame,
                      #highlightbackground="black",highlightthickness=1, bd = 2,
                      image = white_img,
                      height= 40, width = 85,
                      bg = 'white'
                      )
gamepad_icon = Label(upperRight_frame,
                      #highlightbackground="black",highlightthickness=1, bd = 2,
                      image = gamepad_img,
                      height= 40, width = 40,
                      bg = 'white'
                     )
dist_label = Label(upperRight_frame,
                      #highlightbackground="black",highlightthickness=1, bd = 2,
                      text = 'Distance \ntraveled:',
                      bg = 'white'
                     )
odomDist_label = Label(upperRight_frame,
                      #highlightbackground="black",highlightthickness=1, bd = 2,
                      text = '0.00m',
                      bg = 'white'
                     )
vel_label = Label(upperRight_frame,
                      #highlightbackground="black",highlightthickness=1, bd = 2,
                      text = 'Velocity:',
                      bg = 'white'
                     )
velValue_label = Label(upperRight_frame,
                      #highlightbackground="black",highlightthickness=1, bd = 2,
                      text = '0.00 m/s',
                      bg = 'white'
                     )


joystick_button = Button(left_frame, 
                        highlightbackground=borderColor,highlightthickness=2,
                        bd = 2 ,
                        height= button_size, width = button_size, image = joystick_on_img,
                        bg = 'white', fg = 'black',
                        command = joystick_callback)

autoParking_button = Button(left_frame, 
                        highlightbackground=borderColor,highlightthickness=2,
                        bd = 2 ,
                        height= button_size, width = button_size, image = autoParking_img,
                        bg = 'white', fg = 'black',
                        command = autoParking_callback)


gripperOpen_button = Button(left_frame, 
                        highlightbackground=borderColor,highlightthickness=2,
                        bd = 2 ,
                        height= button_size, width = button_size, image = openArrow_img,
                        bg = 'white', fg ='black') #, 
                        #command = gripperOpen_callback)

gripperClose_button = Button(left_frame, 
                        highlightbackground=borderColor,highlightthickness=2,
                        bd = 2 ,
                        height= button_size, width = button_size, image = closeArrow_img,
                        bg = 'white', fg ='black') #,  
                        #command = gripperClose_callback)

gripperOpen_button.bind('<ButtonPress-1>',gripperOpen_callback)
gripperOpen_button.bind('<ButtonRelease-1>',gripperStop_callback)
gripperClose_button.bind('<ButtonPress-1>',gripperClose_callback)
gripperClose_button.bind('<ButtonRelease-1>',gripperStop_callback)

actuatorUp_button = Button(left_frame, 
                        highlightbackground=borderColor,highlightthickness=2,
                        bd = 2 ,
                        height= button_size, width = button_size, image = upArrow_img,
                        bg = 'white', fg ='black') #, 
                        #command = actuatorUp_callback)

actuatorDown_button = Button(left_frame, 
                        highlightbackground=borderColor,highlightthickness=2,
                        bd = 2 ,
                        height= button_size, width = button_size, image = downArrow_img,
                        bg = 'white', fg ='black') #, 
                        #command = actuatorDown_callback)

actuatorUp_button.bind('<ButtonPress-1>',actuatorUp_callback)
actuatorUp_button.bind('<ButtonRelease-1>',actuatorStop_callback)
actuatorDown_button.bind('<ButtonPress-1>',actuatorDown_callback)
actuatorDown_button.bind('<ButtonRelease-1>',actuatorStop_callback)

roscore_button = Button(left_frame, 
                        highlightbackground="red",highlightthickness=1,bd = 2 ,
                        height= 2, width = 12, text = 'Start ROS Master', fg ='red', 
                        command = roscore_callback)

quit_button = Button(upperRight_frame, 
                     #highlightbackground="black",highlightthickness=1,
                     bd = 2 ,
                     height= 40, width = 40, image = exitIcon_img, 
                     bg = 'green', fg ='black',
                     command = quit)


#======================Pack and Grid=========================================

#--------------------------upper frame-------------------


#--------------------------upper Right frame-------------
def displayUpperRightFrame():
    vel_label.grid(row = 0, column = 0, sticky = W, padx = 5, pady =2)
    velValue_label.grid(row = 0, column = 1, sticky = W, padx = 5, pady =2)
    dist_label.grid(row = 0, column = 2, sticky = W, padx = 5, pady =2)
    odomDist_label.grid(row = 0,column = 3,sticky = W, padx = 5,pady = 2)
    gamepad_icon.grid(row = 0,column = 4,sticky = W, padx = 5,pady = 2)
    carParking_icon.grid(row = 0,column = 5,sticky = W, padx = 5,pady = 2)
    battery_label.grid(row = 0, column = 6,sticky = W, padx = 5,pady = 2)
    emergency_icon.grid(row = 0, column = 7, sticky = W, padx = 5, pady = 2)
    quit_button.grid(row = 0, column = 8, sticky = W, padx = 5,pady = 2)


#--------------------------left frame--------------------

def displayLeftFrame():
    joystick_button.grid(row = 0,column = 0,sticky = W, padx = 5,pady = 2)
    autoParking_button.grid(row = 1,column = 0,sticky = W, padx = 5,pady = 2)
    gripperClose_button.grid(row = 2,column = 0, sticky = W, padx = 5, pady = 2)
    gripperOpen_button.grid(row = 3,column = 0, sticky = W, padx = 5, pady = 2)
    actuatorUp_button.grid(row = 4,column = 0, sticky = W, padx = 5, pady = 2)
    actuatorDown_button.grid(row = 5,column = 0, sticky = W, padx = 5, pady = 2)
    # roscore_button.grid(row = 1, column = 0, sticky = W, pady = 2)


def update():
    global start_time
    global is_joy_turn_on
    # global displayUpperRightFrame
    # global displayLeftFrame
    
    while True:
########################### BATTERY ###############################
        battery_is_charging = rospy.wait_for_message(batteryTopic, battery, timeout=1).is_charging
        # print(battery_is_charging)
        if battery_is_charging == True:
            carParking_icon.config(image = carParking_img)
        else:
            carParking_icon.config(image = white_img)
########################### EM ####################################
        EM_is_pressed = rospy.wait_for_message(emergencyTopic, Bool, timeout=1).data
        if EM_is_pressed == True:
            emergency_icon.config(image = emergency_img)
        else:
            emergency_icon.config(image = white_img)

########################### TIME #################################
        t = str(time.asctime(time.localtime(rospy.get_time())))
        # time_test.set(t)
        root.update_idletasks()
        root.update()
        displayUpperRightFrame()
        displayLeftFrame()


#====================main window==============================================

def start():
    
    rospy.init_node('UI_test')
    
    global pub_servo_cmd
    global pub_actuator_cmd
    global pub_charge_cmd
    global pub_joystick_activate
    global start_time
    global emergency_icon
    global carParking_icon
    global gamepad_icon
    
    #--------------------Publisher-----------------
    pub_servo_cmd = rospy.Publisher(gripperTopic, String, queue_size = 10)    
    pub_actuator_cmd = rospy.Publisher(actuatorTopic, String, queue_size = 10)
    pub_charge_cmd = rospy.Publisher(autoChargingTopic, String, queue_size = 1)
    pub_joystick_activate = rospy.Publisher(joystickTopic, Bool, queue_size = 1)
    #--------------------Subscriber----------------
    sub_odomDist = rospy.Subscriber(odomDistTopic, String, odomDistCallback)
    sub_vel = rospy.Subscriber(odomTopic, Odometry, velCallback)
    
    start_time = time.time()
    emergency_icon = Label(upperRight_frame,image = white_img, bg = 'white', height= 40, width = 40)
    carParking_icon = Label(upperRight_frame,image = white_img, bg = 'white', height= 40, width = 40)
        
    displayUpperRightFrame()
    displayLeftFrame()
    
    
    bg = threading.Thread(target=eye_stream, args=(background_label,))
    bg.daemon = 1
    bg.start()
    
    refresh = threading.Thread(target=update)
    refresh.daemon = 1
    refresh.start()
    
    battery = threading.Thread(target=battery_stream, args=(battery_label,))
    battery.daemon = 1
    battery.start()
    
    root.mainloop()
    root.destroy()

if __name__ == "__main__":
    start()
