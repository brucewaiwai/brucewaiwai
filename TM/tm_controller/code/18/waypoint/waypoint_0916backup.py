#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import time
import math
import csv
import threading
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, PoseWithCovarianceStamped
from playsound import playsound
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Path

path = 'path5.csv'
current_map = 'AA_1'


class waypoint:

    def __init__(self):
        # ros config
        self.sub_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback,  queue_size = 1)
        self.sub_cmd = rospy.Subscriber('/tm/goal',String,self.goal_callback)
        self.move_base = rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.move_base_result)
        self.sub_global_plan = rospy.Subscriber('/move_base/GlobalPlanner/plan',Path,self.global_plan_callback)

        self.pub_server_cmd = rospy.Publisher('/tm/server_cmd', String, queue_size=10)
        self.pub_task_state = rospy.Publisher('/tm/task_state', String, queue_size=10)
        self.pub_charge_cmd = rospy.Publisher('/start_charging', String, queue_size=10)
        self.pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 5)


        self.path = []
        self.next_point_dis = 3.5
        self.point = 0
        self.state = 'idle'
        self.total_loop = 0
        self.num_loop = 10
        self.charge_period = 600

        self.playingsound = False
        self.current_map = current_map
        self.global_plan = 1
        self.global_plan_count = 0

        self.move_base_data = 0

    def global_plan_callback(self,data):
        self.len_global_plan = len(data.poses)
        if self.len_global_plan == 0:
            self.global_plan_count += 1
        else:
            self.global_plan_count = 0

        if self.global_plan_count >= 10:
            self.global_plan = 0
            # self.global_plan_count = 0
        else:
            self.global_plan = 1
            
            
        # print(self.global_plan)
    
    def move_base_result(self,data):
        self.move_base_data = data.status.status


    def Reset_position(self,pos):
        if pos == 'origin':
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = 1
            pose.pose.pose.position.y = 0
            pose.pose.pose.orientation.z = 0
            pose.pose.pose.orientation.w = 1
            self.pub_pose.publish(pose)

        elif pos == 'AA_2':
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = 318.9
            pose.pose.pose.position.y = 8.81
            pose.pose.pose.orientation.z = -1
            pose.pose.pose.orientation.w = 0
            self.pub_pose.publish(pose)

        elif pos == 'AA_1':
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = -6.48
            pose.pose.pose.position.y = -292.29
            pose.pose.pose.orientation.z = 0.67
            pose.pose.pose.orientation.w = 0.739
            self.pub_pose.publish(pose)


    def change_map(self,MAP):
        # if MAP == 'AA_1':
        try:
            cmd = 'cp ~/Desktop/map/' + MAP + '/* ~/Desktop/map/'
            os.system(cmd)
            print('change map to ' + MAP)
            rospy.sleep(2)
            cmd = 'rosnode kill /loc_map /nav_map'
            os.system(cmd)
            rospy.sleep(5)

            if self.current_map == 'AA_3':
                self.Reset_position('AA_2')

            elif self.current_map == 'AA_1':
                self.Reset_position('origin')

            elif self.current_map == 'AA_2':
                if MAP == 'AA_3':
                    self.Reset_position('origin')
                elif MAP == 'AA_1':
                    self.Reset_position('AA_1')
            # cmd = String()
            # cmd.data = 'Reset_position'
            # self.pub_server_cmd.publish(cmd)

            self.current_map = MAP
        except Exception as e:
            print('change map error' + e)


    def pose_callback(self,data):
        # print(data)
        # pass
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.pose_z = round(math.atan(data.pose.pose.orientation.z/data.pose.pose.orientation.w) * 2 ,2)

        # print(self.pose_x,self.pose_y,self.pose_z)


    def load_point(self):
        try:

            with open(path) as csvfile:
                rows = csv.reader(csvfile)
                
                self.path = [row for row in rows]
                print('loaded path',self.path)

            # print(self.path)
        except Exception as e:
            print(e)



    def pub_state(self,event):
        cmd = String()
        cmd.data = self.state
        self.pub_task_state.publish(cmd)


    def move(self):

        # while 1:
            # if self.played_woop == False:
            #     self.played_woop = True


            if self.state != 'charging':
                self.loop = 0

                # while self.loop < self.num_loop:
                i = 0
                self.skiped_pt = 0
                while True:
                    
                    # x, y, z, point_cmd, target map
                    try:
                        pt_id = float(self.path[self.point][0])
                        goal_x = float(self.path[self.point][1])
                        goal_y = float(self.path[self.point][2])
                        goal_z = float(self.path[self.point][3])
                        point_cmd = self.path[self.point][4]
                        target_map = self.path[self.point][5]
                    except:

                        pass


                    # print('1',goal_x,goal_y,goal_z)
                    # print('2',self.pose_x,self.pose_y,self.pose_z)
                    # if i == 0:
                    cmd = String()
                    cmd.data = "StartMove:" + str(goal_x) + "," + str(goal_y) + "," + str(goal_z)
                    self.pub_server_cmd.publish(cmd)

                        
                        



                    #################################
                    dis_to_goal = math.hypot(self.pose_x - goal_x, self.pose_y - goal_y)
                    angdis_to_doal = abs(abs(self.pose_z) - abs(goal_z))

                    if dis_to_goal <= self.next_point_dis:
                        self.skiped_pt = 0
                        self.state = 'running'
                        
                        if point_cmd == 'stop':
                            # if dis_to_goal < 0.5 and angdis_to_doal < 0.15:
                            if self.move_base_data == 3:
                                self.state = 'arrived'
                                rospy.sleep(20)
                                self.point += 1
                        elif point_cmd == 'stopandchangemap':
                            # if dis_to_goal < 0.5 and angdis_to_doal < 0.15:
                            if self.move_base_data == 3:
                                self.state = 'arrived'
                                rospy.sleep(5)
                                self.change_map(target_map)
                                rospy.sleep(5)
                                self.point += 1

                        else:
                            self.state = 'running'
                            self.point += 1
                    
                    else:
                        if point_cmd == '0':
                            if self.global_plan == 0:
                                # self.global_plan = 1
                                
                                print('no plan...')
                                if self.skiped_pt <= 4:
                                    self.point += 1
                                    self.skiped_pt += 1
                                    print('skip point')
                                self.state = 'no_plan'
                            else:
                                self.state = 'running'
                                self.skiped_pt = 0

                        else:
                            if self.global_plan == 0:
                                # self.global_plan = 1
                                
                                print('no plan...')
                                self.state = 'no_plan'
                            else:
                                self.state = 'running'
                                self.skiped_pt = 0


                    if self.point >= len(self.path):
                        # self.point = 0
                        # self.loop += 1
                    ##################################

                        self.state = 'idle'
                        break

                    # print(self.point,cmd)
                    
                    print('----------------')
                    print('dis',dis_to_goal)
                    print('ang',angdis_to_doal)
                    print('state',self.state)
                    print(cmd)
                    print('point', pt_id)
                    print('skip_pt',self.skiped_pt)
                    print('blocked count',self.global_plan_count)
                    # print('loop',self.loop)
                    # print('total_loop',self.total_loop)
                    i += 1
                    rospy.sleep(1)

            # self.total_loop += 1
            # if self.total_loop == 10:


            ######charge
                # print('wait for charge')
                # rospy.sleep(10)
                # self.state = 'charging'
                # self.start_charge_time = rospy.get_time()

                
            #     cmd = String()
            #     cmd = 'start'
            #     self.pub_charge_cmd.publish(cmd)

            # if self.state == 'charging':
            #     charging_time = rospy.get_time() - self.start_charge_time
            #     print('charging time > 600 break',charging_time)
                
            #     if charging_time > self.charge_period:
            #         self.state = 'idle'
            #         cmd = String()
            #         cmd = 'stop'
            #         self.pub_charge_cmd.publish(cmd)
            #         print('stop charge')
            #         self.total_loop += 1
            #         rospy.sleep(10)

            #     rospy.sleep(1)

            










    def goal_callback(self,data):

        if data.data == 'go':
            move = threading.Thread(target=self.move)
            move.daemon = 1
            move.start()
        # else data.data == 'skip':
            

            
if __name__ == '__main__':

    rospy.init_node('waypoint')

    ic = waypoint()
    # rospy.loginfo('START')

    ic.load_point()
    rospy.Timer(rospy.Duration(1), ic.pub_state)
    rospy.spin()

