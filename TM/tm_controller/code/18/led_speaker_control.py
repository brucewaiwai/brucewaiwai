#!/usr/bin/env python
import threading
import rospy
from playsound import playsound
from std_msgs.msg import ColorRGBA, String, Bool, Float32
from geometry_msgs.msg import Twist
from tm_controller.msg import battery
from tm_controller.msg import delta


led_state = [0,0,0,0]
led_cmd = [0,0,0,0]

light_state = [0,0,0,0]
light_cmd = [0,0,0,0]




class led:

    def __init__(self):


        rospy.Subscriber('/state/Led_state', ColorRGBA, self.led_callback)
        rospy.Subscriber('/state/EM_button', Bool, self.em_callback)
        # rospy.Subscriber('/load/loading_state', String, self.loading_callback)
        # rospy.Subscriber('/battery/info', battery, self.battery_callback)
        rospy.Subscriber('/state/delta/state', String, self.delta_callback)
        rospy.Subscriber('/tm/inside_footprint',Bool, self.footprint_callback)
        rospy.Subscriber('/tm/task_state', String, self.task_callback)
        rospy.Subscriber('/state/battery/info', battery, self.battery_status_callback)

        self.pub_led = rospy.Publisher('/base_control/Led_cmd', ColorRGBA, queue_size = 5)


        self.EM_button = False
        self.is_charging = False
        self.led_state = []
        self.played = False
        self.inside_footprint = False
        self.battery_level = 0.0
        self.battety_level_low = 30.0
        self.battety_level_high = 70.0
        self.played_woop = False
        self.playingsound = False
        self.robot_state = 'idle'
    

    def task_callback(self,data):
        self.robot_state = data.data



    def led_callback(self, data):
        color_r = data.r
        color_g = data.g
        color_b = data.b
        color_mode = data.a
        self.led_state = [color_r,color_g,color_b,color_mode]


    def em_callback(self,data):

        self.EM_button = data.data

    def delta_callback(self, data):

        if data.data == 'charging':
            self.is_charging = True
        else:
            self.is_charging = False

    # def battery_callback(self, data):

    #     self.is_charging = data.is_charging

    def footprint_callback(self, data):
        
        self.inside_footprint = data.data

    def battery_status_callback(self, data):
        
        self.battery_level = data.battery_level


    # def play_woop(self):
    #     playsound('/home/tmrobot/catkin_ws/src/TM/start_robot/sound/woop.mp3')
    #     self.played_woop = False
        

# loading_state = False
# def loading_callback(data):
#     global loading_state
#     if data.data != 'unloaded' and data.data!= "loaded_and_merged":
#         loading_state = True
#     else:
#         loading_state = False


    def play_mp3(self,MP3):

        playsound('/home/tmrobot/catkin_ws/src/TM/start_robot/sound/' + MP3 + '.mp3')
        self.playingsound = False
        
    def play_sound(self,MP3):

        if self.playingsound == False:
            self.playingsound = True
            mp3 = threading.Thread(target=self.play_mp3,args=(MP3,))
            mp3.daemon = 1
            mp3.start()


    def led_control(self,event):


        if self.EM_button == True:#em button on
            self.led(255,255,0,1)

        else: #em button off

            if self.is_charging == True:
                #self.led(0,255,0,0)

                if self.battery_level <= self.battety_level_low:
                    #print("Battery level is low")
                    self.led(255,0,0,0)  #Red
                elif self.battery_level >= self.battety_level_high:
                    #print("Battery level is high")
                    self.led(0,255,0,0)  #Green
                elif self.battety_level_low < self.battery_level < self.battety_level_high:
                    #print("Battery level is medium")
                    self.led(255,255,0,0)   #yelllow
                
                if self.played == False:
                    self.played = True
                    playsound('/home/tmrobot/catkin_ws/src/TM/start_robot/sound/bootup_0002.ogg')

            else:
                if self.inside_footprint == True:
                    self.led(255,0,0,1)
                    self.play_sound('woop')
                
                    # if self.played_woop == False:
                    #     self.played_woop = True
                    #     mp3 = threading.Thread(target=self.play_woop)
                    #     mp3.start()
                else:    
                    if self.robot_state == 'running':
                        self.play_sound('beep')
                    elif self.robot_state == 'arrived':
                        self.play_sound('arrived')
                    elif self.robot_state == 'no_plan':
                        self.play_sound('excuse_me')

                    
                    self.led(0,255,150,0)
                    self.played = False
                



    def led(self,r,g,b,a):
        led_cmd = [r,g,b,a]
        cmd = ColorRGBA()
        cmd.r = r
        cmd.g = g
        cmd.b = b
        cmd.a = a
        if self.led_state != led_cmd:
            self.pub_led.publish(cmd)
        # rospy.loginfo(str(cmd))



if __name__ == '__main__':

    rospy.init_node('led_control')

    ic = led()
    rospy.Timer(rospy.Duration(0.5), ic.led_control)
    sound_path = '/home/tmrobot/catkin_ws/src/TM/start_robot/sound/'

    playsound(sound_path + 'bootup_0001.ogg')
    # rospy.loginfo('START')
    # ic.load_point()

    rospy.spin()


