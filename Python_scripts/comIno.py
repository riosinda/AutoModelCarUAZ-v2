#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import serial, time

class datos:
    
  def __init__(self):
    self.auto_vel = rospy.Publisher('auto/vel', Int16, queue_size=10)
    self.auto_ste = rospy.Publisher('auto/ste', Int16, queue_size=10)
    self.auto_rpm = rospy.Publisher('auto/rpm', Int16, queue_size=10)
    self.auto_dist = rospy.Publisher('auto/dist', Int16, queue_size=10)
    self.auto_ax = rospy.Publisher('ax', Int16, queue_size=10)
    self.auto_ay = rospy.Publisher('ay', Int16, queue_size=10)
    self.auto_az = rospy.Publisher('az', Int16, queue_size=10)
    self.auto_gx = rospy.Publisher('gx', Int16, queue_size=10)
    self.auto_gy = rospy.Publisher('gy', Int16, queue_size=10)
    self.auto_gz = rospy.Publisher('gz', Int16, queue_size=10)
    rospy.init_node('tuCola', anonymous=True)
    #self.serArduino = serial.Serial("/dev/ttyUSB1",115200,timeout=1)

  def sendNodes(self):
    if not rospy.is_shutdown():
      self.auto_ax.publish(self.auto_ax_str)
      self.auto_ay.publish(self.auto_ay_str)
      self.auto_az.publish(self.auto_az_str)
      self.auto_gx.publish(self.auto_gx_str)
      self.auto_gy.publish(self.auto_gy_str)
      self.auto_gz.publish(self.auto_gz_str)
      self.auto_vel.publish(self.auto_vel_str)
      self.auto_dist.publish(self.auto_dist_str)
  
  def receiveNodes(self, datos):
    #vector = self.serArduino.read().split()
    #print(vector)
    vector = datos
    if not rospy.is_shutdown():
       self.auto_ax_str = vector[0]
       self.auto_ay_str = vector[1]
       self.auto_az_str = vector[2]
       self.auto_gx_str = vector[3]
       self.auto_gy_str = vector[4]
       self.auto_gz_str = vector[5]
       self.auto_vel_str = vector[6]
       self.auto_dist_str = vector[7]
        
if __name__ == '__main__':
    sw=0
    try:
        while True:
            serArduino = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
            vector = serArduino.readline()
            vector = vector.split(',')
            
            print(vector)
            print(len(vector))
            print(sw)
            sw = sw + 1
            
            if len(vector) == 8:
                ic = datos()
                ic.receiveNodes(vector)
                ic.sendNodes()
                rospy.spin()
    
    except rospy.ROSInterruptException:
        pass