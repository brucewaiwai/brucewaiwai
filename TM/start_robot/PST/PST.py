#!/usr/bin/env python
#-*- coding: utf-8 -*-
#Version: 1.0 
print('===== Version: 1.0 =====')
from Tkinter import *
import subprocess
import sys

root = Tk()
PATH = '/home/tmrobot/catkin_ws/src/'

# def start_generate_map():
#     subprocess.call("gnome-terminal -x bash -ic 'roslaunch cartographer_ros mapping.launch'", shell=True)

def start_generate_map():
    subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/start_robot/start_script; sh start_mapping.sh'", shell=True)

def continue_map():
    subprocess.call("gnome-terminal -x bash -ic 'roslaunch cartographer_ros continue_mapping.launch'", shell=True)

def start_all():
    subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/start_robot/start_script; sh start.sh'", shell=True)

def start_rviz():
    subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/deploy/rviz; rviz -d mapping.rviz'", shell=True)

def save_map():
    subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/start_robot/start_script; sh save_map.sh'", shell=True)

# def save_map():
#     subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/deploy/map/map; rosrun map_server map_saver -f map'", shell=True)

def start_rviz_nav():
    subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/deploy/rviz; rviz -d nav.rviz'", shell=True)

def start_control_panel():
    subprocess.call("gnome-terminal -x bash -ic 'cd /home/tmrobot/catkin_ws/src/TM/start_robot/PST; python control_panel.py'", shell=True)

def exit_mapping():
    sys.exit(0)

if __name__ == "__main__":
    root.geometry("700x550")
    root.title("TM_AGV")
    Label(root, text="TM mapping", width = 10, height = 3, font=("blue", 10)).pack()
    big_fm = Frame(root)

    fm1 = Frame(big_fm)
    Label(fm1, text="Mapping", width = 10, height = 3, font=("blue", 10)).pack()

    start_cartographer = Button(fm1, text = "Start Mapping", width = 20, height = 4, command = start_generate_map)
    continue_mapping = Button(fm1, text = "Continue Mapping", width = 20, height = 4, command = continue_map)
    save_map = Button(fm1, text = "Save Map", width = 20, height = 4, command = save_map)

    fm1.pack(side=LEFT, fill=BOTH, expand=YES)
    start_cartographer.pack()
    continue_mapping.pack()
    save_map.pack()

    

    fm2 = Frame(big_fm)
    Label(fm2, text="Start Robot", width = 10, height = 3, font=("blue", 10)).pack()
    start_robot = Button(fm2, text = "Start", width = 20, height = 4, command = start_all)
    rviz = Button(fm2, text = "Rviz", width = 20, height = 4, command = start_rviz_nav)
    control_panel = Button(fm2, text = "Control_panel", width = 20, height = 4, command = start_control_panel)
    fm2.pack(side=LEFT, fill=BOTH, expand=YES)
    start_robot.pack()
    rviz.pack()
    control_panel.pack()



    exit_button = Button(root, text = "Quit", width = 30, height = 4, command = exit_mapping)
    big_fm.pack(side=TOP)
    exit_button.pack(side=BOTTOM)
    root.mainloop()

    
