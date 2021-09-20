#!/usr/bin/env python
import rospy
import math
import time
import os
from datetime import datetime
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import String

#PATH = "/home/ken/catkin_ws/src/kinrobot_test"
PATH = "/home/tmrobot/catkin_ws/src/TM/tm_controller/UI_test"

class OdometryModifier:

  def __init__(self):
    self.odomSub = rospy.Subscriber("odom", Odometry, self.odomCallback)
    self.odomResetSub = rospy.Subscriber("odom_dist_reset", Bool, self.resetCallback)
    self.odomDistPub = rospy.Publisher('odom_dist', String, queue_size=10)
    self.total_distance = 0.0
    self.previous_x = 0.0
    self.previous_y = 0.0
    self.first_run = True
    self.now = datetime.now()
    self.date_time = self.now.strftime("%m/%d/%Y, %H:%M:%S")
    self.last_line = ""
    self.printedOnce = False
    self.preiousTime = self.now

  def odomCallback(self, data):
    if(self.first_run):
      self.previous_x = data.pose.pose.position.x
      self.previous_y = data.pose.pose.position.y
      self.first_run = False
      # print(self.first_run)
      self.readDataFromFile()
      self.total_distance = float(self.last_line.split()[2])

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    d_increment = math.sqrt((x - self.previous_x) * (x - self.previous_x) + (y - self.previous_y)*(y - self.previous_y))
    if d_increment >= 0.00001:
      self.total_distance = self.total_distance + d_increment
    self.odomDistPub.publish(str(self.total_distance))
    self.previous_x = x
    self.previous_y = y
    print("=======================================")
    print(self.last_line)
    self.now = datetime.now()
    self.date_time = self.now.strftime("%m/%d/%Y, %H:%M:%S")

    if self.now.hour%4 == 0 and self.printedOnce == False:
      self.writeDataToFile()
      self.printedOnce = True
      self.preiousTime = self.now
    # else:
    #   print("Odom will be saved every 4 hours...")
    
    if self.now.hour == (self.preiousTime.hour + 4) or self.now.hour == (self.preiousTime.hour - 20): #every 4 hrs
      self.printedOnce = False

    # print("Time:",self.date_time)
    # print(self.first_run)
    # print("previous_x: %f" %self.previous_x)
    # print("previous_y: %f" %self.previous_y)
    # print("x: %f" %x)
    # print("y: %f" %y)
    # print("d_increment: %f" %d_increment)
    print("Total distance traveled is {%f}m" %self.total_distance)

    

  def resetCallback(self, data):
    if data.data == True:
      FileHandler = open(PATH + "/odomList.txt","r+")
      FileHandler.truncate(0)
      FileHandler.close()
      self.total_distance = 0.0
      self.previous_x = 0.0
      self.previous_y = 0.0
      self.last_line = "0 0 0.0"
      self.first_run = True
  
  def readDataFromFile(self):
    FileHandler = open(PATH + "/odomList.txt","r")
    if os.stat(PATH + "/odomList.txt").st_size != 0:
      self.last_line = FileHandler.readlines()[-1]
    else:
      self.total_distance = 0.0
    FileHandler.close()
    


  def writeDataToFile(self):
    FileHandler = open(PATH + "/odomList.txt","a+")
    FileHandler.write(self.date_time + "\t")
    FileHandler.write(str(self.total_distance) + "\n")
    FileHandler.close()

if __name__ == '__main__':
  try:
    rospy.init_node('show_odom', anonymous=False)
    odom = OdometryModifier()
    rospy.spin()
    odom.writeDataToFile()
  except rospy.ROSInterruptException:
    pass