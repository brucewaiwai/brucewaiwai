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


class dynamic_footprint:

    def __init__(self):

        rospy.Subscriber('/state/Actuator_state', Float64, self.actuator_state_callback)


        self.footprint_mode = 'withlinen'


    def actuator_state_callback(self,data):
        if data.data > 0:
            
            if self.footprint_mode == 'withoutlinen':
                self.footprint_mode = 'withlinen'
                self.change_footprint(self.footprint_mode)

            # self.footprint_mode = 'withlinen'
                
        elif data.data == 0:
            if self.footprint_mode == 'withlinen':
                self.footprint_mode = 'withoutlinen'
                self.change_footprint(self.footprint_mode)

            # self.footprint_mode = 'withoutlinen'

    def change_footprint(self,cmd):

        # print('test')
        client_F = dynamic_reconfigure.client.Client("/laser_filter_sick_F/polygon_filter", timeout=30, config_callback=None)
        client_B = dynamic_reconfigure.client.Client("/laser_filter_sick_B/polygon_filter", timeout=30, config_callback=None)
        client_R = dynamic_reconfigure.client.Client("/laser_filter_sick_R/polygon_filter", timeout=30, config_callback=None)
        client_L = dynamic_reconfigure.client.Client("/laser_filter_sick_L/polygon_filter", timeout=30, config_callback=None)
        client_FF = dynamic_reconfigure.client.Client("/laser_filter_s1_F/polygon_filter", timeout=30, config_callback=None)
        client_BB = dynamic_reconfigure.client.Client("/laser_filter_s1_B/polygon_filter", timeout=30, config_callback=None)
        
        s1_withlinen = '[[-0.75, -0.46], [-0.75, 0.36], [0.75, 0.36], [0.75, -0.46]]'
        s1_withoutlinen = '[[-0.76, -0.46], [-0.76, -0.34], [-0.35, -0.34], [-0.35, 0.40], [0.35, 0.40], [0.35, -0.34], [0.76, -0.34], [0.76, -0.46]]'
        sick_withlinen = '[[-0.75, -0.46], [-0.75, 0.35], [0.75, 0.35], [0.75, -0.46]]'
        sick_withoutlinen = '[[-0.75, -0.46], [-0.75, -0.33], [-0.34, -0.33], [-0.34, 0.35], [0.34, 0.35], [0.34, -0.33], [0.75, -0.33], [0.75, -0.46]]'

        if cmd == 'withoutlinen':
            s1_polygon = s1_withoutlinen
            sick_polygon = sick_withoutlinen
        elif cmd == 'withlinen':
            s1_polygon = s1_withlinen
            sick_polygon = sick_withlinen


        client_F.update_configuration({"polygon":sick_polygon})
        client_B.update_configuration({"polygon":sick_polygon})
        client_R.update_configuration({"polygon":sick_polygon})
        client_L.update_configuration({"polygon":sick_polygon})
        client_FF.update_configuration({"polygon":s1_polygon})
        client_BB.update_configuration({"polygon":s1_polygon})



def start():

 
    rospy.init_node('tm_controller')

    ic = dynamic_footprint()


    rospy.spin()

if __name__ == '__main__':
    start()

