#!/usr/bin/env python
# from art import *
#"===== Version: 1.0 07/06/2021 ====="
# tprint("TMrobot")

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float64
from std_msgs.msg import Bool
import threading
import time

linear_max_speed = 1
linear_min_speed = 0.1
angular_max_speed = 1
angular_min_speed = 0.1




class joystick:
    def __init__(self,linear_speed,angular_speed):

        self.pub_cmd_x = rospy.Publisher('/cmd_vel_x', Twist, queue_size = 5)
        self.pub_actuator_cmd = rospy.Publisher('/base_control/Actuator_cmd', String, queue_size = 10)
        self.pub_servo_cmd = rospy.Publisher('/base_control/Servo', String, queue_size = 10)
        self.pub_ser_cmd = rospy.Publisher('/tm/server_cmd', String, queue_size = 5)
        self.pub_load_cmd = rospy.Publisher('/load/control_cmd', String, queue_size = 5)
        self.pub_charge_cmd = rospy.Publisher('/start_charging', String, queue_size = 1)

        rospy.Subscriber("/tm/inside_footprint", Bool, self.footprint_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/turn_on_joy_controller", Bool, self.turn_on_joy_callback)
        rospy.Subscriber("/tm/stop", Bool, self.stop_cmd)
        # rospy.Subscriber("/state/Actuator_state", Float64, self.actuator_callback)
        rospy.Subscriber("/joy_cheat", Joy, self.joy_cheat_callback)
#
        # self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        ##config
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.enable_button = 0
        self.mode = 0


        self.button_list = []
        self.de_speed_up = False
        self.de_speed_down = False
        self.de_mode = False
        self.inside_footprint = True
        self.cheat_enable = False
        self.turn_on_joy = True

        self.time_start = rospy.get_time()

        self.stop_cmd = False
    # def actuator_callback(self,data):
    #     self.actuator_pos = data.data
    def stop_cmd(self,data):
        self.stop_cmd = data.data


    def turn_on_joy_callback(self,data):
        self.turn_on_joy = data.data

    def debounce(self, args):

        time.sleep(0.5)

        if args == 'button_speed_up':
            self.de_speed_up = False
        if args == 'button_speed_down':
            self.de_speed_down = False
        if args == 'button_mode':
            self.de_mode = False


    def footprint_callback(self,data):
        self.inside_footprint = data.data

    # def timer(self):

    def joy_cheat_callback(self,data):

        # if self.cheat_enable == False:
        time_now = rospy.get_time()
        twist = Twist()
        if data.buttons[7] == 1:
            self.cheat_enable = True
            self.time_start = rospy.get_time()
            twist.linear.x = 0.2 * data.axes[1]
            # twist.linear.y = self.linear_speed * data.axes[3]
            twist.angular.z = 0.2 * data.axes[0]
            self.pub_cmd_x.publish(twist)

        elif data.buttons[7] == 0:
            self.cheat_enable = False
        
        if time_now - self.time_start >= 3:
            self.cheat_enable = False
            # twist.linear.x = self.linear_speed * data.axes[0]
            # twist.linear.y = self.linear_speed * data.axes[3]
            # twist.angular.z = self.linear_speed * data.axes[1]
            # self.pub_cmd_x.publish(twist)

        print(self.cheat_enable)

    def joy_callback(self, data):
        

    ##################################


        
        button_A = data.buttons[0]
        button_B = data.buttons[1]
        button_X = data.buttons[2]
        button_Y = data.buttons[3]
        button_L1 = data.buttons[4]
        button_R1 = data.buttons[5]
        button_back = data.buttons[6]
        button_start = data.buttons[7]
        button_home = data.buttons[8]
        button_L3 = data.buttons[9]
        button_R3 = data.buttons[10]
        

        # for i in range(len(data.buttons)):
        #     button_id = i
        #     button_state = data.buttons[i]
        #     button = BUTTON(button_id,button_state)

        #     self.check_button(button)

        # for but in self.button_list:
            
        #     if but.id == 0:
        #         button_A = but.press
        #     elif but.id == 1:
        #         button_B = but.press
                
        # print(button_A,button_B)


        axe_L2 = data.axes[2]
        axe_R2 = data.axes[5]

    ###################################
        twist = Twist()
        if self.stop_cmd == False:
            if self.cheat_enable == False:
                if button_L1 == 1:
                    if button_Y == 1:
                        cmd = String()
                        cmd.data = 'up'
                        self.pub_actuator_cmd.publish(cmd)
                        rospy.loginfo(str(cmd))

                    elif button_A == 1:

                        cmd = String()
                        cmd.data = 'down'
                        self.pub_actuator_cmd.publish(cmd)
                        rospy.loginfo(str(cmd))

                    elif button_X == 1:
                        cmd = String()
                        cmd.data = 'close'
                        self.pub_servo_cmd.publish(cmd)
                        rospy.loginfo('servo on')

                    elif button_B == 1:
                        cmd = String()
                        cmd.data = 'open'
                        self.pub_servo_cmd.publish(cmd)
                        rospy.loginfo('servo off')
                    
                    if self.turn_on_joy == True:
                        self.enable_button = True
                        if self.mode == 0:
                            twist.linear.x = self.linear_speed * data.axes[1]
                            twist.linear.y = self.linear_speed * data.axes[3]
                            twist.angular.z = self.linear_speed * data.axes[0]
                            self.pub_cmd_x.publish(twist)
                        elif self.mode == 1:
                            twist.linear.x = -self.linear_speed * data.axes[1]
                            twist.linear.y = -self.linear_speed * data.axes[3]
                            twist.angular.z = self.linear_speed * data.axes[0]
                            self.pub_cmd_x.publish(twist)


                elif button_L1 == 0:
                    # cmd = String()
                    # cmd.data = 'stop'
                    # pub_actuator_cmd.publish(cmd)
                    
                    if self.enable_button == True:
                        self.enable_button = False
                        twist.linear.x = 0
                        twist.linear.y = 0
                        twist.angular.z = 0
                        self.pub_cmd_x.publish(twist)
                    # rospy.loginfo(str(cmd))
        else:
            self.stop()
            # print('stop')

        if axe_L2 == -1:  
            if button_Y == 1:
                cmd = String()
                cmd.data = 'vup'
                self.pub_actuator_cmd.publish(cmd)
                rospy.loginfo(str(cmd))

            elif button_A == 1:
                cmd = String()
                cmd.data = 'vdown'
                self.pub_actuator_cmd.publish(cmd)
                rospy.loginfo(str(cmd))

            elif button_X == 1:
                cmd = String()
                cmd.data = 'vclose'
                self.pub_servo_cmd.publish(cmd)

            elif button_B == 1:
                cmd = String()
                cmd.data = 'vopen'
                self.pub_servo_cmd.publish(cmd)
    ####################################



    ###################load cmd
        if button_R1 == 1 :
            if button_X == 1:##button X
                cmd = String()
                cmd.data = 'StartLoad'
                self.pub_ser_cmd.publish(cmd)
                rospy.loginfo(str(cmd))

            elif button_B == 1:##button B
                cmd = String()
                cmd.data = 'StartUnLoad'
                self.pub_ser_cmd.publish(cmd)
                rospy.loginfo(str(cmd))

            elif button_Y == 1:##button Y
                cmd = String()
                cmd.data = 'StopLoad'
                self.pub_ser_cmd.publish(cmd)
                rospy.loginfo(str(cmd))
                
    ########################change speed
        if self.de_speed_down == False:
            if button_R1 == 1 and axe_L2 == -1:#button A
                self.de_speed_down = True
                

                if self.linear_speed > linear_min_speed:
                    self.linear_speed = round(self.linear_speed - 0.1 , 2)
                    rospy.loginfo('speed'+ str(self.linear_speed))

                if self.angular_speed > angular_min_speed:
                    self.angular_speed = round(self.angular_speed - 0.2 , 2)

                    
                threading.Thread(target = self.debounce, args = ('button_speed_down',)).start()

        if self.de_speed_up == False:
            if button_R1 == 1 and axe_R2 == -1:##button Y
                self.de_speed_up = True

                if self.linear_speed < linear_max_speed:
                    self.linear_speed = round(self.linear_speed + 0.1 ,2)
                    rospy.loginfo('speed'+ str(self.linear_speed))

                if self.angular_speed < angular_max_speed:
                    self.angular_speed = round(self.angular_speed + 0.2 ,2)

                threading.Thread(target = self.debounce, args = ('button_speed_up',)).start()

    ########################change mode
        if self.de_mode == False:
            if button_R3 == 1:
                self.de_mode = True
                if self.mode == 0:
                    self.mode = 1
                elif self.mode == 1:
                    self.mode = 0
                rospy.loginfo('mode: ' + str(self.mode))

                threading.Thread(target = self.debounce, args = ('button_mode',)).start()
        
        if button_start == 1:
            cmd = String()
            cmd = 'start'
            self.pub_charge_cmd.publish(cmd)
            # print(mode)

    #########################################################








    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.angular.z = 0
        self.pub_cmd_x.publish(twist)




    # def cmd_vel_callback(self,data):
    #     if self.enable_button == False:
    #         self.pub_cmd_x.publish(data)


    def cmd_vel_callback(self,event):

    # def cmd_vel_callback(self,data):


        if self.enable_button == False and self.cheat_enable == False and self.stop_cmd == False:

            try:


                # if self.inside_footprint == False and self.cheat_enable == False:
                if self.inside_footprint == False:
                    try:
                        data = rospy.wait_for_message("/cmd_vel", Twist, timeout=1)
                        self.pub_cmd_x.publish(data)
                        print('navigating...')
                    except Exception as e:
                        print('nav error',e)
                        self.stop()
                        # pass

                else:
                    self.stop()
            
            except Exception as e:
                self.stop()

                print(e)





def start():

    rospy.init_node('Joy2Wheeltec')
    linear_speed = rospy.get_param("~linear_speed")
    angular_speed = rospy.get_param("~angular_speed")

    ic = joystick(linear_speed,angular_speed)
    rospy.Timer(rospy.Duration(0.2), ic.cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    start()

