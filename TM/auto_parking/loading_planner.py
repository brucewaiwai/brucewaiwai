#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import time
import math
import sympy 
# 2021-06-04

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# from playsound import playsound
import threading



simulation = False

if simulation:
    scan_topic = '/scan'
    tag_topic = '/follow_me/rviz_tag'
    vel_topic = '/cmd_vel'
else:
    scan_topic = '/sick_scan_filtered'
    tag_topic = '/follow_me/leg_tag'
    vel_topic = '/cmd_vel'



path_topic = '/load/path'
cmd_topic = '/follow_me/cmd'
state_topic = '/follow_me/state'


        
class driver:

    def __init__(self):
        # ros configq
        self.sub_scan = rospy.Subscriber(scan_topic, LaserScan, self.laser_callback,  queue_size = 1)
        self.sub_tag = rospy.Subscriber(tag_topic, Point, self.tag_callback,  queue_size = 1)
        self.sub_cmd = rospy.Subscriber(cmd_topic, String, self.cmd_callback,  queue_size = 10)

        self.pub_cmd_vel = rospy.Publisher(vel_topic, Twist, queue_size=10)
        self.pub_path = rospy.Publisher(path_topic, Path, queue_size=1)
        self.pub_state = rospy.Publisher(state_topic, String, queue_size=10)


        # init parameter 
        self.follow_me = False
        self.rotate = False
        self.playingsound = False
        self.state = ''
        
        # robot config
        self.robot_footprint = [[0.4,0.3], [0.4,-0.3], [-0.4,-0.3], [-0.4,0.3]]
        
        self.robot_size = [0.84,0.6]
        self.x = np.array([0.0, 0.0, 0, 0.0, 0.0, 0.0])
        self.predict_time = 2
        # self.ob = np.array([[1,1],
        #                    [-1,1],
        #                    [-1,0],
        #                    [0,1],
        #                    [2,-1],
        #                    [0.5,-1],
        #                    [-0.5,-1]])
        # [x,y,yaw,speed_x,speed_yaw,speed_y]
        


    def cmd_callback(self,data):
        
        if data.data == 'StartFollow':
            if self.follow_me == False:
                self.follow_me = True
                self.state = 'Start_Follow'
                rospy.loginfo('START FOLLOW')

            else:

                self.follow_me = False
                cmd = Twist()
                cmd.linear.x = 0
                cmd.angular.z = 0
                
                self.pub_cmd_vel.publish(cmd)

                self.state = 'Stop_Follow'
                rospy.loginfo('STOP FOLLOW')


    def laser_callback(self,data):
        # print('hi')
        min_dis = 0.0
        max_dis = 2
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angles_range = self.angle_max - self.angle_min
        self.ranges = data.ranges
        self.angle_increment = data.angle_increment
        # self.add_op()
        obstacle = []
        # self.ob = []
        min_angle = -math.pi/2
        max_angle = math.pi/2
        # min_angle_num = int((min_angle - self.angle_min - math.pi/8)/self.angle_increment)
        # max_angle_num = int((max_angle - self.angle_min + math.pi/8)/self.angle_increment)
        try:
            for i in range(len(self.ranges)):
                # if (i <=max_angle_num and i >=min_angle_num):
                ranges = self.ranges[i]
                if ranges < 0.1:
                    ranges = 15

                if ranges >= min_dis and ranges <= max_dis:
                    x = ranges * math.cos (self.angle_min + i * self.angle_increment - 1.57)
                    y = ranges * math.sin (self.angle_min + i * self.angle_increment - 1.57)

                    obstacle.append([x,y])
            self.ob = np.array(obstacle)

        except Exception as e:
            print(e)



    def tag_callback(self,data):
        self.tag_x = data.x
        self.tag_y = data.y
        # self.goal = np.array([self.tag_x, self.tag_y])
        self.goal = np.array([self.tag_x, self.tag_y])


    def calc_dynamic_window(self,x):
        """
        calculation dynamic window based on current state x
        """
        # max_speed = (1 - math.exp(-2 * (dis - 0.7))) * 0.5
        
        min_speed_x = 0.0
        max_speed_x = 0.2

        # min_speed_y = 0.01
        max_speed_y = 0.3

        max_yaw_rate = 0.02

        max_accel_x = 0.2
        max_accel_y = 0.3
        max_delta_yaw_rate = 0.2  # [rad/ss]
        dt = 0.1


        # Dynamic window from robot specification
        # Vs = [min_speed_y, max_speed_y,
        #      -max_yaw_rate, max_yaw_rate]

        Vs = [min_speed_x, max_speed_x,
             -max_yaw_rate, max_yaw_rate,
             -max_speed_y, max_speed_y]

        # Dynamic window from motion model
        Vd = [x[3] - max_accel_x * dt,
              x[3] + max_accel_x * dt,
              x[4] - max_delta_yaw_rate * dt,
              x[4] + max_delta_yaw_rate * dt,
              x[5] - max_accel_y * dt,
              x[5] + max_accel_y * dt,]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3]),
              max(Vs[4], Vd[4]), min(Vs[5], Vd[5])]
        # print('dw',dw)
        return dw


    def calc_to_goal_cost(self, trajectory, goal):# for angle calculation

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))


        return cost

    def calc_to_goal_cost2(self, trajectory, goal):# for dis calculation

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]

        d = math.hypot(dx, dy) - 1.2
        if d < 0:
            return float("inf")
        # d = np.clip(d, 0, 4)

        return d 
        

    def calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        # print('-----------------')
        # print(trajectory)

        rl = self.robot_size[0]/2
        rw = self.robot_size[1]/2

        ox = ob[:, 0]
        oy = ob[:, 1]

        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]

        dxdy = np.array([dx,dy])
        arg = np.where((abs(dxdy[0]) > rl) & (abs(dxdy[1]) > rw))
        r = np.hypot(abs(dxdy[0][arg])-rl, abs(dxdy[1][arg])-rw)

        # print(r)
        d1 = abs(dy[abs(dx)<rl])- rw
        d2 = abs(dx[abs(dy)<rw])- rl


        if np.any(d1<0): # obstacle inside the footprint
            return float("inf")
        if np.any(d2<0):
            return float("inf")
        


        min_r = min(r) if len(r) > 0 else 2
        min_d1 = min(d1) if len(d1) > 0 else 2
        min_d2 = min(d2) if len(d2) > 0 else 2
        # print(min_d1, min_d2, min_r)
        min_d = min(min_d1, min_d2, min_r)


        if abs(min_d) > 0.6:
            min_d = 0.6

        cost = 1/min_d

        if (abs(min_d) <= 0.05):# obstacle inside the footprint

            cost = float("inf")



        # print(cost)

        # return dx, dy, yaw, min_r
        return cost

    def cal_speed_cost(self, max_speed, trajectory):


        cost = (max_speed - trajectory[-1, 3])
        return cost

    def predict_trajectory(self, x_init, v, y, u):
        """
        predict trajectory with an input
        """


        predict_time = self.predict_time
        dt = 0.1
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        # 
        while time <= predict_time:
        # while time < 0.1:
            x = self.motion(x, [v, y, u], dt)
            trajectory = np.vstack((trajectory, x))
            time += dt
        # print(trajectory)
        return trajectory


    def dwa_b_planner(self,x):
        print('-----dwa-----')

        dw = self.calc_dynamic_window(x)
        print(dw)
        best_p = 1
        min_cost = float("inf")

        collision = False

        # if self.dist_to_goal >1.5:

        min_speed_x = dw[0]
        max_speed_x = dw[1]
        min_angular = dw[2]
        max_angular = dw[3]
        min_speed_y = dw[4]
        max_speed_y = dw[5]

        y = 0
        v = 0
        best_trajectory = []
        cal_trajectory = False
        for v in np.arange(min_speed_x, max_speed_x, 0.01): # speed_x
            for u in np.arange(min_speed_y, 0, 0.01): # speed_y
                
                predict_trajectory = self.predict_trajectory(self.x, v, y, u)
                # to_goal_cost = self.calc_to_goal_cost(predict_trajectory, self.goal)  # angular
                # to_goal_cost2 = self.calc_to_goal_cost2(predict_trajectory, self.goal) # dis

                ob_cost = np.array(self.calc_obstacle_cost(predict_trajectory,self.ob))


                speed_cost = self.cal_speed_cost(max_speed_x, predict_trajectory)
                
                total_cost = ob_cost + speed_cost

                # print('u1',u,ob_cost)

                if min_cost > total_cost:
                    min_cost = total_cost
                    cal_trajectory = True
                    # best_p = [ob_cost, speed_cost, to_goal_cost, to_goal_cost2, total_cost]
                    best_p = ob_cost
                    best_u = [v,y,u]
                    best_trajectory = predict_trajectory

            for u in np.arange(max_speed_y, 0, -0.01): # speed_y
                
                predict_trajectory = self.predict_trajectory(self.x, v, y, u)
                # to_goal_cost = self.calc_to_goal_cost(predict_trajectory, self.goal)  # angular
                # to_goal_cost2 = self.calc_to_goal_cost2(predict_trajectory, self.goal) # dis

                ob_cost = np.array(self.calc_obstacle_cost(predict_trajectory,self.ob))


                speed_cost = self.cal_speed_cost(max_speed_x, predict_trajectory)
                
                total_cost = ob_cost + speed_cost

                # print('u2',u,ob_cost)

                if min_cost > total_cost:
                    min_cost = total_cost
                    cal_trajectory = True
                    # best_p = [ob_cost, speed_cost, to_goal_cost, to_goal_cost2, total_cost]
                    best_p = ob_cost
                    best_u = [v,y,u]
                    best_trajectory = predict_trajectory
                    

                    




        if cal_trajectory == False:
            best_u = [0,0,0]
        print('ob',best_p)
        # print('collision ',collision)
        return best_u, best_trajectory

        
    def motion(self, x, u, dt):
        """
        motion model
        """

        

        # x[0] += u[0] * dt
        # x[1] += u[2] * dt
        # x[2] += u[1] * dt

        x[0] += (u[0] * math.cos(x[2]) - u[2] * math.sin(x[2]))* dt
        x[1] += (u[0] * math.sin(x[2]) + u[2] * math.cos(x[2]))* dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        x[5] = u[2]

        return x

    def motion2(self, x, u):
        """
        motion model
        """
        x[3] = u[0] #speed_x
        x[4] = u[1] #speed_yaw
        x[5] = u[2] #speed_y

        return x


    def drive(self):
        try:

            # print('##########################')


            u, predict_trajectory = self.dwa_b_planner(self.x)



            cmd = Path()
            cmd.header.stamp = rospy.get_rostime()
            cmd.header.frame_id = 'base_footprint'
            for i in range(len(predict_trajectory)):
                pose = PoseStamped()
                pose.pose.position.x = -predict_trajectory[i][1]
                pose.pose.position.y = predict_trajectory[i][0]
                cmd.poses.append(pose)
            
            self.pub_path.publish(cmd)


            self.x = self.motion2(self.x, u)

            self.robot_linear_x = u[0]
            self.robot_linear_y = u[2]
            self.robot_angular = u[1]

            # print('robot_linear: %f' %self.robot_linear)
            # print('robot_angular: %f' %self.robot_angular)

            print(u[0],u[2],u[1])


            # cmd = Twist()
            # cmd.linear.y = self.robot_linear_x
            # # cmd.linear.y = 0
            # cmd.linear.x = -self.robot_linear_y
            # cmd.angular.z = self.robot_angular

            # self.pub_cmd_vel.publish(cmd)







        except Exception as e:
            print(e)


def pub_follow_me_state():
    while 1:

        cmd = String()
        cmd.data = ic.state
        ic.pub_state.publish(cmd)
        time.sleep(1)

if __name__ == '__main__':

    rospy.init_node('driver')

    ic = driver()
    # rospy.loginfo('START')



    while not rospy.is_shutdown():

        try:
            if simulation:
                ic.drive()
            else:
                try:
                    ic.drive()
                except Exception as e:
                    print(e)

                
        except KeyboardInterrupt:
            print('quit')
            break
