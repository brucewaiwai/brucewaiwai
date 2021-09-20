#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped


class Autopark:
    def __init__(self):

        ## Change those variables can change the parking position
        self.target_pos_x = 0.088
        self.target_pos_y = -0.051
        self.target_ori_x = -0.498
        self.target_ori_y = 0.517
        self.target_ori_z = 0.488
        self.target_ori_w = -0.495
        
        ## Change those variables can change the tolerance of parking and speed of parking
        self.pos_x_tolerance = 0.005
        self.pos_y_tolerance = 0.005
        self.ori_tolerance = 0.02
        self.vel_x = 0.03
        self.vel_y = 0.03
        self.ang_z = 0.02

        ## Unchange variables
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.ori_x = 0.0
        self.ori_y = 0.0
        self.ori_z = 0.0
        self.ori_w = 0.0
        self.marker = ''
        self.command =''
        self.cmd = ''
        self.marker_found = False
        self.start = False
        self.x_done = False
        self.y_done = False
        self.z_done = False
        self.is_charging = False
        self.leave = False
        self.time_interval = 5
        self.time_previous = 0
        self.exc = False
    def ar_marker_callback(self, data):
        self.marker = data.markers
        if not data.markers:
            print("No marker found")
            self.marker_found = False
        else:
            self.marker_found = True

        if self.marker_found == True:
            if self.marker[0].id == 4:
                self.pos_x = self.marker[0].pose.pose.position.x
                self.pos_y = self.marker[0].pose.pose.position.y
                self.ori_x = self.marker[0].pose.pose.orientation.x
                self.ori_y = self.marker[0].pose.pose.orientation.y
                self.ori_z = self.marker[0].pose.pose.orientation.z
                self.ori_w = self.marker[0].pose.pose.orientation.w
        ##Start the auto-park when receive the command
        if self.start == True:
            self.auto_tune()
        

    def start_charging_callback(self, data):
        self.command = data.data
        if self.command == 'start':
            self.start = True
            rospy.loginfo("Start Parking")
        elif self.command =='stop':
            self.start = False
            self.leave =True
            self.exc = False
            while self.leave:
                self.leave_wireless_charger()
            else:
                rospy.loginfo("Stopped Charging")
        else:
            self.start = False

    def is_charging_callback(self, data):
        self.cmd = data.data
        if self.cmd == 'charging':
            self.is_charging = True
        else:
            self.is_charging = False
    
    def leave_wireless_charger(self):
        vel = Twist()
        if time.time() - self.time_previous > self.time_interval:
            self.time_previous = time.time()
            if self.exc == True:
                vel.linear.y = 0
                self.pub_vel.publish(vel)
                self.leave = False
            elif self.exc == False:
                vel.linear.y = self.vel_y * 4
                self.pub_vel.publish(vel)
                # if self.is_charging == False :
                self.exc = True
            
    def auto_tune(self):
        vel = Twist()
        ## The Algorithm is trying to tune the horizontal direction first to prevent the lost of tracking
        ## Then adjust the vertical and angular direction
        if self.pos_y - self.target_pos_y < -10 * self.pos_y_tolerance:
            vel.linear.x = -3*self.vel_x
            vel.angular.z = self.tune_angle(self.target_ori_y, self.target_ori_z)
            self.y_done = False
        elif -self.pos_y_tolerance > (self.pos_y - self.target_pos_y) > -10 * self.pos_y_tolerance:
            vel.linear.x = -self.vel_x
            vel.linear.y = self.tune_distance(self.target_pos_x)
            vel.angular.z = self.tune_angle(self.target_ori_y, self.target_ori_z)
            self.y_done = False
        elif self.pos_y - self.target_pos_y > 10 * self.pos_y_tolerance:
            vel.linear.x = 3*self.vel_x
            vel.angular.z = self.tune_angle(self.target_ori_y, self.target_ori_z)
            self.y_done = False
        elif self.pos_y_tolerance < (self.pos_y - self.target_pos_y) < 10 * self.pos_y_tolerance:
            vel.linear.x = self.vel_x
            vel.linear.y = self.tune_distance(self.target_pos_x)
            vel.angular.z = self.tune_angle(self.target_ori_y, self.target_ori_z)
            self.y_done = False
        else:
            vel.linear.x = 0.0
            vel.linear.y = self.tune_distance(self.target_pos_x)
            vel.angular.z = self.tune_angle(self.target_ori_y, self.target_ori_z)
            self.y_done = True
            print("y done")    
        if self.marker_found == False:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            print("Lost tracking")
        if (self.x_done == True ) and (self.y_done == True) and (self.z_done == True):
            if self.is_charging == True:
                self.start = False
                print("Finished Parking and Charging")
            else:
                print("Finished Parking")
        else:
            print("Parking")

        self.pub_vel.publish(vel)

    def tune_distance(self, position_x):
        if self.pos_x - position_x > 10*self.pos_x_tolerance:
            lin_vel = -self.vel_y * 1.5
            self.x_done = False
        elif self.pos_x_tolerance < self.pos_x - position_x < 10*self.pos_x_tolerance:
            lin_vel = -self.vel_y * 0.5
            self.x_done = False
        # elif self.pos_x - position_x > 10*self.pos_x_tolerance:
        #     lin_vel = self.vel_y * 1.5
        #     self.x_done = False
        # elif -self.pos_x_tolerance > self.pos_x - position_x > -10*self.pos_x_tolerance:
        #     lin_vel = self.vel_y * 0.5
        #     self.x_done = False
        else:
            lin_vel = 0.0
            self.x_done = True
            print("x done")
        return lin_vel

    def tune_angle(self, angle_y, angle_z):

        if(0 <= (abs(self.ori_y) - abs(angle_y)) < self.ori_tolerance) and (0 <= (abs(self.ori_z) - abs(angle_z)) < self.ori_tolerance):
            ang_vel = 0.0
            self.z_done = True
            print("z done")
        elif(-self.ori_tolerance < (abs(self.ori_y) - abs(angle_y)) <= 0) and (-self.ori_tolerance < (abs(self.ori_z) - abs(angle_z)) <= 0 ):
            ang_vel = 0.0
            self.z_done = True
            print("z done")
        elif(abs(self.ori_y) - abs(angle_y) > 5 * self.ori_tolerance) or (abs(self.ori_z) - abs(angle_z) > 5* self.ori_tolerance):
            ang_vel = -3 * self.ang_z
            self.z_done = False

        elif(abs(self.ori_y) - abs(angle_y) < -5 * self.ori_tolerance) or (abs(self.ori_z) - abs(angle_z) < -5 * self.ori_tolerance):
            ang_vel = 3 * self.ang_z
            self.z_done = False

        elif(abs(self.ori_y) - abs(angle_y) > self.ori_tolerance) or (abs(self.ori_z) - abs(angle_z) > self.ori_tolerance):
            ang_vel = -self.ang_z
            self.z_done = False

        elif(abs(self.ori_y) - abs(angle_y) < -self.ori_tolerance) or (abs(self.ori_z) - abs(angle_z) < -self.ori_tolerance):
            ang_vel = self.ang_z
            self.z_done = False

        else:
            ang_vel = 0
            # print("Angle error")

        return ang_vel
            

            
    def main(self):
        self.sub_ar_tag = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_marker_callback)
        self.sub_start_charging = rospy.Subscriber('/start_charging', String, self.start_charging_callback)
        self.sub_is_charging = rospy.Subscriber('/delta/state', String, self.is_charging_callback)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)



        # # rate = rospy.Rate(0.1)
        
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('auto_parking')
    callclass = Autopark()
    callclass.main()

