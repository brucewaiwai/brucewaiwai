#!/usr/bin/env python
from __future__ import division

# print('===== Version 1.4 =====')
# print('2021-08-19')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from tm_controller.msg import battery


import numpy as np
import datetime
import serial
import time
import sys
import matplotlib.pyplot as plt
import datetime


data_voltage = []
data_current = []
data = []
xtime = []
data_t1 = []
data_t2 = []


fig, ax = plt.subplots(3, 1, figsize=(9,8))


def s16(value):
    return -(value & 0x8000) | (value & 0x7fff)

class BATTERY:
    def __init__(self):

        self.ser = serial.Serial('/dev/TMbattery', 9600, timeout=2)  # '/dev/ttyACM1'
        # self.ser = serial.Serial('/dev/ttyUSB3', 9600, timeout=2)
        # if self.ser.isOpen() :
        #     print("open success")
        # else :
        #     print("open failed")
        
        self.pub_battery_info = rospy.Publisher('/state/battery/info', battery, queue_size = 1)
        # self.pub_battery_voltage = rospy.Publisher('/battery/voltage', Float32, queue_size = 5)


    def readser(self,event):

        
        cmd1 = [0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 ]
        cmd2 = [0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77 ]
        
        try:
            self.ser.write(cmd1)

            read = self.ser.read(34)
            if(read[0] == bytearray(b'\xdd') and read[-1] == bytearray(b'\x77')):
                read = read.encode('hex')
                # print(read)

                v = read[8] + read[9] + read[10] + read[11]
                voltage = round((int(v, 16) / 100),2)

                c = read[12] + read[13] + read[14] + read[15]
                
                current = round(s16(int(c,16))/100,2)
                # print(type(current))
                # if current >= 0:

                # current = (65536 - int(c,16)) / 100


                power = round(voltage * current,2)


                remain_c = read[16] + read[17] + read[18] + read[19]

                remain_capacity = int(remain_c,16) * 10

                
                rated_c = read[20] + read[21] + read[22] + read[23]
                rated_capacity = int(rated_c,16) * 10

                battery_level1 = round(remain_capacity/rated_capacity * 100,2)
                battery_level2 = int(read[46] + read[47], 16)
                battery_level3 = (voltage - 23)/ (29.4 - 23) * 100
                cell = int(read[50] + read[51], 16)

                temperature1 = (int(read[54] + read[55] + read[56] + read[57],16) - 2731) / 10
                temperature2 = (int(read[58] + read[59] + read[60] + read[61],16) - 2731) / 10
                

                

                cmd = battery()
                cmd.voltage = voltage
                cmd.current = current
                cmd.remain_capacity = remain_capacity
                cmd.rated_capacity = rated_capacity
                cmd.battery_level = battery_level1
                cmd.is_charging = True if current >=0 else False
                cmd.temperature1 = temperature1
                cmd.temperature2 = temperature2
                self.pub_battery_info.publish(cmd)

                # cmd  = Float32
                # cmd.data = voltage
                # self.pub_battery_voltage.publish(cmd)


                t = datetime.datetime.now()
                fo = open("/home/tmrobot/catkin_ws/src/TM/tm_controller/log.txt", "a+")
                log = '\n' + str(t) + ' ' + str(temperature1) + ' ' + str(temperature2)+ ' ' + str(voltage) + ' ' + str(current) + ' ' + str(battery_level1)
                fo.write(log)
                
                print('-------------------------------------------')
                print('voltage: %g V' %voltage)
                print('current: %g A' %current)
                print('power: %g W' %power)
                print('remain_capacity: %g mAh' %remain_capacity)
                print('rated_capacity: %g mAh' %rated_capacity)
                # print('battery_level1: %g %%' %battery_level1)
                # print('battery_level2: %g %%' %battery_level2)
                print('battery_level: %g %%' %battery_level1)
                # print('cell: %g ' %cell)
                print('temperature1: %g ' %temperature1)
                print('temperature2: %g ' %temperature2)


            # cells voltage
            # self.ser.write(cmd2)
            # read = self.ser.read(21)
            # cell = []
            # if(read[0] == bytearray(b'\xdd') and read[-1] == bytearray(b'\x77')):
            #     read = read.encode('hex')
            #     # print(read)
            #     j = 0
            #     for i in range(7):
            #         cell_v = int(read[j + 8] + read[j + 9] + read[j + 10] + read[j + 11],16)
            #         print('cell %i voltage: %i'%(i+1 ,cell_v))
            #         # cell.append(cell_v)
            #         j = j + 4



            
            
            # xtime.append(datetime.datetime.now())
            # data_voltage.append(voltage)
            # data_current.append(current)
            # data_t1.append(temperature1)
            # data_t2.append(temperature2)

            # if len(xtime) > 600:
            #     xtime.pop(0)
            #     data_voltage.pop(0)
            #     data_current.pop(0)
            #     data_t1.pop(0)
            #     data_t2.pop(0)

            # ax[0].cla()
            # ax[1].cla()

            # ax[0].plot(xtime, data_voltage, 'b')
            # ax[0].set_title('Voltage',fontsize=10)
            # ax[0].grid(True)
            # ax[0].tick_params(axis='x', labelrotation=20)

            # ax[1].plot(xtime, data_current, 'r')
            # ax[1].set_title('Current',fontsize=10)
            # ax[1].grid(True)
            # ax[1].tick_params(axis='x', labelrotation=20)

            # ax[2].plot(xtime, data_t1, 'r')
            # ax[2].plot(xtime, data_t2, 'g')
            # ax[2].set_title('Temperature',fontsize=10)
            # ax[2].grid(True)
            # ax[2].tick_params(axis='x', labelrotation=20)
            # ax[2].legend(['t1','t2'])

            # plt.pause(0.1)
        except Exception as e:
            print(e)
            # print(self.ser)
            try:
                self.ser.close()
                self.ser = serial.Serial('/dev/TMbattery', 9600, timeout=2)
            except Exception as e:
                print(e)

            # print(self.ser.isOpen())
            # if self.ser.isOpen():
                # print('test2')
            #     self.ser.close()
            #     time.sleep(1)
            #     self.ser.open()

  

if __name__ == '__main__':
    #### start robot log
    
    t = datetime.datetime.now()
    # print(str(t))
    fo = open("/home/tmrobot/catkin_ws/src/TM/tm_controller/start_robot_log.txt", "a+")
    log = '\n' + str(t)
    fo.write(log)
    time.sleep(1)
    ####

    rospy.init_node('battery')
    record_rate = rospy.get_param("~record_rate",1)

    ic = BATTERY()
    rospy.Timer(rospy.Duration(1/record_rate), ic.readser)
    rospy.spin()
