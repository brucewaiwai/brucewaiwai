#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import String
from tm_controller.msg import delta
from tm_controller.msg import battery
import serial
import struct
import cantools
import threading
import time
import math


db = cantools.db.load_file('/home/tmrobot/catkin_ws/src/TM/tm_controller/delta/Charger_CAN_Protocol_V1.13.dbc')



class message:
    def __init__(self, id, data):
        self.id = id
        self.data = data



class DELTA:
    def __init__(self, port, baudrate,cut_off_voltage,charging_current):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

        self.port = port
        self.baudrate = baudrate

        self.WIRELESS_STATUS1 = 0
        self.BATTERY_VOLTAGE1 = 0
        self.CHARGING_CURRENT1 = 0
        self.FAULT_STATUS1 = 0
        self.POWER_STATUS1 = 0
        self.BATTERY_TEM1 = 0
        self.CHARGER_TEM1 = 0
        self.WIRELESS_STATUS2 = 0
        self.BATTERY_VOLTAGE2 = 0
        self.CHARGING_CURRENT2 = 0
        self.FAULT_STATUS2 = 0
        self.POWER_STATUS2 = 0
        self.BATTERY_TEM2 = 0
        self.CHARGER_TEM2 = 0

        self.cut_off_voltage = cut_off_voltage
        self.charging_current = charging_current

        self.msg_list = []
        self.charging = False


        self.charging_state = 'idel'
        self.reboot_count = 0
        self.receive_msg = 0
        self.battery_level = 0
        self.current = 0

        # rospy
        self.pub_charging_state = rospy.Publisher('/state/delta/state', String, queue_size = 1)
        self.pub_delta_info = rospy.Publisher('/state/delta/info', delta, queue_size = 1)

        rospy.Subscriber('/battery/info', battery, self.battery_callback)


    def battery_callback(self,data):
        self.battery_level = data.battery_level


    def check_msg(self, msg):
        # print(msg.id)
        for message in self.msg_list:
            if message.id == msg.id:
                message.data = msg.data
                return

        self.msg_list.append(msg)


    def charge_cmd(self,arg):


        # voltage = self.BATTERY_VOLTAGE1

        cut_off_voltage = self.cut_off_voltage

        # current = self.charging_current

        if self.battery_level < 90:

            current = self.charging_current

        elif self.battery_level >= 90:

            current = 30 * (math.exp(- 0.23 * (self.battery_level - 90)))

            if current > self.charging_current:
                current = self.charging_current

        self.current = current


        start_charge_msg = db.encode_message(400,{'Demand_PowerStage9': 0, 'Demand_PowerStage8': 0, 
                                                'Demand_PowerStage7': 0, 'Demand_PowerStage6': 0,
                                                'Demand_PowerStage5': 0, 'Demand_PowerStage4': 0,
                                                'Demand_PowerStage3': 0, 'Demand_PowerStage2': 0,
                                                'Demand_PowerStage10': 0, 'Demand_ClearFaults': 0,
                                                'Demand_PowerStage1': 1, 'Demand_Voltage': cut_off_voltage, 'Demand_Current': current})

        stop_charg_msg = db.encode_message(400,{'Demand_PowerStage9': 0, 'Demand_PowerStage8': 0, 
                                                'Demand_PowerStage7': 0, 'Demand_PowerStage6': 0,
                                                'Demand_PowerStage5': 0, 'Demand_PowerStage4': 0,
                                                'Demand_PowerStage3': 0, 'Demand_PowerStage2': 0,
                                                'Demand_PowerStage10': 0, 'Demand_ClearFaults': 0,
                                                'Demand_PowerStage1': 0, 'Demand_Voltage': cut_off_voltage, 'Demand_Current': current})

        clear_fault_msg = db.encode_message(400,{'Demand_PowerStage9': 0, 'Demand_PowerStage8': 0, 
                                                'Demand_PowerStage7': 0, 'Demand_PowerStage6': 0,
                                                'Demand_PowerStage5': 0, 'Demand_PowerStage4': 0,
                                                'Demand_PowerStage3': 0, 'Demand_PowerStage2': 0,
                                                'Demand_PowerStage10': 0, 'Demand_ClearFaults': 1,
                                                'Demand_PowerStage1': 0, 'Demand_Voltage': cut_off_voltage, 'Demand_Current': current})

        head = struct.pack('>6Bh',0xaa,0x00,0x00,0x07,0x00,0x00,0x0190)

        start_charge_data = head + start_charge_msg
        stop_charge_data = head + stop_charg_msg
        clear_fault_data = head + clear_fault_msg


        if arg == 'start':
            data = start_charge_data
            # self.ser.write(start_charge_data)
        elif arg == 'stop':
            data = stop_charge_data
            # self.ser.write(stop_charge_data)
        elif arg == 'clear_fault':
            data = clear_fault_data
            # self.ser.write(clear_fault_data)

        self.cmd = data
        


    def read_msg(self,event): 

        
        try:
            if self.ser == None:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

            else:    
                read = self.ser.read()
                # print(read)
                if(read == bytearray(b'\xaa')):
                    msg_ = read + self.ser.read(15)

                    # print(msg_.encode('hex'))

                    if len(msg_) != 16:
                        print('test')
                        print(msg_.encode('hex'))
                        self.ser.close()
                        time.sleep(1)
                        self.ser.open()

                    else:
                        # print(msg_.encode('hex'))
                        
                        msg_id = ''
                        msg_data = ''

                        for i in range(4,8): msg_id += msg_[i]
                        
                            
                        msg_id = int(msg_id.encode('hex'),16)
                        # print(msg_id)
                        # if msg_id == 786:
                            # self.receive_msg += 1
                        if msg_id == 1521 or msg_id == 1522 or msg_id == 785 or msg_id == 786 or msg_id == 1441 or msg_id == 1442:
                            
                        
                            for i in range(8,16): msg_data += msg_[i]

                            msg = message(msg_id,msg_data)
                            self.check_msg(msg)

        except Exception as e:
            print(e)
            try:
                if self.ser != None:
                    self.ser.close()
                    self.ser = None
                    
            except Exception as e:
                print(e)



    def send_msg(self,event):


        if self.ser != None:
            try:
                if self.WIRELESS_STATUS1 == 1 or self.WIRELESS_STATUS2 == 1:
                    self.ser.write(self.cmd)
                    
            except Exception as e:
                print(e)




                
    def decode(self,event):
        # print('----------------')
        for msg in self.msg_list:
            # print(msg.id)
            ##charger 1
            if msg.id == 1521: # WirelessStatusReport_1

                data = db.decode_message(msg.id, msg.data)
                self.WIRELESS_STATUS1 = data['Status_Wireless_Comms']

            
            elif msg.id == 785: # ModulePowerMeasurementsSlow_1

                data = db.decode_message(msg.id, msg.data)
                self.BATTERY_VOLTAGE1 = data['Report_Meas_Voltage_Slow']
                self.CHARGING_CURRENT1 = data['Report_Meas_Current_Slow']
                self.FAULT_STATUS1 = data['Report_StatusOfFaultSlow']
                self.POWER_STATUS1 = data['Report_StatusOfPowerSlow']

                # print(data)
            
            elif msg.id == 1441:
                data = db.decode_message(msg.id, msg.data)
                self.BATTERY_TEM1 = data['ChargerBatteryTemperature']
                self.CHARGER_TEM1 = data['ChargerTemperature']

            #charger 2
            elif msg.id == 1522: # WirelessStatusReport_1

                data = db.decode_message(msg.id, msg.data)
                self.WIRELESS_STATUS2 = data['Status_Wireless_Comms']

            
            elif msg.id == 786: # ModulePowerMeasurementsSlow_1

                data = db.decode_message(msg.id, msg.data)
                self.BATTERY_VOLTAGE2 = data['Report_Meas_Voltage_Slow']
                self.CHARGING_CURRENT2 = data['Report_Meas_Current_Slow']
                self.FAULT_STATUS2 = data['Report_StatusOfFaultSlow']
                self.POWER_STATUS2 = data['Report_StatusOfPowerSlow']

                # print(data)
            
            elif msg.id == 1442:
                data = db.decode_message(msg.id, msg.data)
                self.BATTERY_TEM2 = data['ChargerBatteryTemperature']
                self.CHARGER_TEM2 = data['ChargerTemperature']


                    
        try:
            # cmd = String()
            # self.charge_cmd('start')

            if self.WIRELESS_STATUS1 or self.WIRELESS_STATUS2 == 1:


                if self.FAULT_STATUS1 == 1 and self.FAULT_STATUS2 == 1:
                    self.charge_cmd('clear_fault')
                    self.charging_state = 'clearing_fault'
                    # rospy.sleep(0.5)

                else:
                    # if self.BATTERY_VOLTAGE1 < self.cut_off_voltage:
                    if self.battery_level < 98:
                        self.charge_cmd('start')
                        self.charging_state = 'charging'

                    else:
                        self.charge_cmd('stop')
                        self.charging_state = 'full'

                # self.charging_state = 'connected'

            else:
                self.charging_state = 'disconnected'




            

        except Exception as e:
            print(e)


        try:
            print('############################################')
            print('cut_off_voltage: %g' %self.cut_off_voltage)
            print('demand_current: %g' %self.charging_current)
            print('current: %g' %self.current)
            print('charging_mode: %g' %charging_mode)

            

            # print('charging_state: %g' %self.charging_state)

            print('-----------------------------charger1')
            print('WIRELESS_STATUS: %g' %self.WIRELESS_STATUS1)
            print('FAULT_STATUS: %g' %self.FAULT_STATUS1)
            print('POWER_STATUS: %g' %self.POWER_STATUS1)
            print('BATTERY_VOLTAGE: %g' %self.BATTERY_VOLTAGE1)
            print('CHARGING_CURRENT: %g' %self.CHARGING_CURRENT1)
            print('BATTERY_TEM: %g' %self.BATTERY_TEM1)
            print('CHARGER_TEM: %g' %self.CHARGER_TEM1)
            

            print('----------------------------charger2')
            print('WIRELESS_STATUS: %g' %self.WIRELESS_STATUS2)
            print('FAULT_STATUS: %g' %self.FAULT_STATUS2)
            print('POWER_STATUS: %g' %self.POWER_STATUS2)
            print('BATTERY_VOLTAGE: %g' %self.BATTERY_VOLTAGE2)
            print('CHARGING_CURRENT: %g' %self.CHARGING_CURRENT2)
            print('BATTERY_TEM: %g' %self.BATTERY_TEM2)
            print('CHARGER_TEM: %g' %self.CHARGER_TEM2)

        except Exception as e:
            print(e)

    def pub_state(self,event):

        cmd = String()
        cmd = self.charging_state
        self.pub_charging_state.publish(cmd)

        cmd = delta()
        cmd.cut_off_voltage = self.cut_off_voltage
        cmd.demand_current = self.charging_current
        cmd.total_current = self.CHARGING_CURRENT1 + self.CHARGING_CURRENT2
        cmd.wireless_status1 = self.WIRELESS_STATUS1
        cmd.wireless_status2 = self.WIRELESS_STATUS2
        cmd.fault_status1 = self.FAULT_STATUS1
        cmd.fault_status2 = self.FAULT_STATUS2
        cmd.power_status1 = self.POWER_STATUS1
        cmd.power_status2 = self.POWER_STATUS2
        cmd.battery_voltage1 = self.BATTERY_VOLTAGE1
        cmd.battery_voltage2 = self.BATTERY_VOLTAGE2
        cmd.charging_current1 = self.CHARGING_CURRENT1
        cmd.charging_current2 = self.CHARGING_CURRENT2
        cmd.charger_tem1 = self.CHARGER_TEM1
        cmd.charger_tem2 = self.CHARGER_TEM2
        cmd.battery_tem = self.BATTERY_TEM1

        self.pub_delta_info.publish(cmd)


    

if __name__ == '__main__':
     
    rospy.init_node('delta')
    
    cut_off_voltage = rospy.get_param("~cut_off_voltage",29)
    charging_current = rospy.get_param("~charging_current",30)
    charging_mode = rospy.get_param("~charging_mode",1)
    port = rospy.get_param("~port",'/dev/TMdelta')

    if charging_mode == 0: #single
        total_current = charging_current

    elif charging_mode == 1:#double
        total_current = charging_current/2


    charger = DELTA('/dev/TMdelta',115200,cut_off_voltage,total_current)

    print('cut_off_voltage: %g' %cut_off_voltage)
    print('demand_current: %g' %charging_current)


    
    rospy.Timer(rospy.Duration(0.5), charger.decode)
    rospy.Timer(rospy.Duration(0.005), charger.read_msg)
    rospy.Timer(rospy.Duration(0.1), charger.send_msg)
    rospy.Timer(rospy.Duration(1), charger.pub_state)
    rospy.spin()
    # pub_state = threading.Thread(target = charger.pub_state)
    # pub_state.daemon=1
    # pub_state.start()

    # while not rospy.is_shutdown():
    #     charger.read_msg()



