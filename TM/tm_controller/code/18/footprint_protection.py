#! /usr/bin/env python
import rospy
import numpy as np
import time
from math import *
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan



class footprint_protection:

    def __init__(self,scan_topic,msg_topic,footprint,footprint_padding):

        
        self.subscriber = rospy.Subscriber(scan_topic, LaserScan, self.laser_callback,  queue_size = 1)
        self.pub_msg = rospy.Publisher(msg_topic, Bool, queue_size = 1)
        self.footprint = footprint

        # find corner of polygon
        self.x_min = 0
        self.x_max = 0
        self.y_min = 0
        self.y_max = 0

        for i in footprint:
            self.x_max = max(i[0],self.x_max)
            self.x_min = min(i[0],self.x_min)
            self.y_max = max(i[1],self.y_max)
            self.y_min = min(i[1],self.y_min)
        
        self.x_min -= footprint_padding
        self.x_max += footprint_padding
        self.y_min -= 0.1
        self.y_max += 0.1

        # print(self.x_min,self.x_max,self.y_min,self.y_max)
    def laser_callback(self,data):
        num_of_point = 0
        
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges

        inside_footprint = False
        for i in range(len(self.ranges)): #laserscan to pointcloud
            x = self.ranges[i] * cos (self.angle_min + i * self.angle_increment)
            y = self.ranges[i] * sin (self.angle_min + i * self.angle_increment)

            if (self.x_min<= x <= self.x_max) and (self.y_min <= y <= self.y_max):
                num_of_point += 1

        if num_of_point >= 2:
            inside_footprint = True
                
        cmd = Bool()
        cmd.data = inside_footprint
        self.pub_msg.publish(cmd)
        # print(inside_footprint)

        



  


if __name__ == '__main__':

    rospy.init_node('footprint_protection')
    
    laserscan_topic = rospy.get_param("~laserscan_topic",'/sick_s1_scan_merged')
    msg_topic = rospy.get_param("~msg_topic",'/tm/inside_footprint')
    footprint = rospy.get_param("~footprint",[[-0.75, -0.45], [-0.75, 0.35],[0.75,0.35],[0.75, -0.45]])
    footprint_padding = rospy.get_param("~footprint_padding",0.2)

    ic = footprint_protection(laserscan_topic,msg_topic,footprint,footprint_padding)


    rospy.spin()

