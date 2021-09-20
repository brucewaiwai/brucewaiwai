#!/usr/bin/env python
# from art import *
# print("===== Version: 1.0 14/12/2020 =====")
# tprint("TMrobot")
import rospy
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Joy
from math import *
import subprocess
import sys

import dynamic_reconfigure.client


class tm_server_cmd:

    def __init__(self):

        rospy.Subscriber('/tm/server_cmd', String, self.cmd_callback)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 5)
        self.pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 5)




    def cmd_callback(self,data):
        cmd = data.data
        
        if cmd == "Reset_position":

            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = 0
            pose.pose.pose.position.y = 0
            pose.pose.pose.orientation.z = 0
            pose.pose.pose.orientation.w = 1
            self.pub_pose.publish(pose)

        elif cmd == "Reset_position2":

            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = 37.23
            pose.pose.pose.position.y = 0.155
            pose.pose.pose.orientation.z = 1
            pose.pose.pose.orientation.w = 0
            self.pub_pose.publish(pose)

        elif cmd == 'ChangeMap':
            rospy.loginfo('ChangMap')
            subprocess.call("gnome-terminal -x bash -ic 'rosnode kill /loc_map /nav_map'", shell=True)

        # elif cmd == 'change_footprint':
        #     self.change_footprint()
        #     print('change footprint')

        else:
            str1 = cmd.split(":", 2)[0]
            str2 = cmd.split(":", 2)[1]

            if str1 == 'StartMove':
                try: 
                    goal_x = float(str2.split(",", 2)[0])
                    goal_y = float(str2.split(",", 2)[1])
                    goal_z = float(str2.split(",", 2)[2])
                
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = goal_x
                    pose.pose.position.y = goal_y
                    pose.pose.orientation.z = sin(goal_z/2)
                    pose.pose.orientation.w = cos(goal_z/2)
                    self.pub_goal.publish(pose)
                    
                except Exception as e:
                    print(e)

    # def click_callback(data):

                
    #                 pose = PoseStamped()
    #                 pose.header.frame_id = 'map'
    #                 pose.header.stamp = rospy.Time.now()
    #                 pose.pose.position.x = goal_x
    #                 pose.pose.position.y = goal_y
    #                 pose.pose.orientation.z = sin(goal_z/2)
    #                 pose.pose.orientation.w = cos(goal_z/2)
    #                 pub_goal.publish(pose)


def start():

 
    rospy.init_node('tm_controller')

    ic = tm_server_cmd()



    # click_sub = rospy.Subscriber('/clicked_point', PointStamped, click_callback)




    rospy.spin()

if __name__ == '__main__':
    start()

