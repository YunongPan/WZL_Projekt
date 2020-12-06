#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('demonstrator_preprocessing')
import time
import math
import tf
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan

class Demonstrator():
  def __init__(self):
    self.rosbot_sub = rospy.Subscriber("/laserscan/front", LaserScan, self.scan_callback)
    self.listener = tf.TransformListener()

    self.angle_min = -1.570796
    self.angle_max = 3.1415926
    self.sample = 720
    self.angle_increment = (self.angle_max - self.angle_min) / self.sample   
 
    self.nearest_point_range = 0.0
    self.nearest_point_index = 0.0
    self.nearest_point_angle = 0.0
    self.nearest_point_x = 0.0
    self.nearest_point_y = 0.0
    self.nearest_point_to_chassis_x = 0.0
    self.nearest_point_to_chassis_y = 0.0
    self.nearest_point_to_chassis = 0.0

    self.ctrl_c = False
    self.rate = rospy.Rate(1)
    rospy.on_shutdown(self.shutdownhook)
	
  def scan_callback(self, msg):
    self.nearest_point_range = min(msg.ranges)
    self.nearest_point_index = msg.ranges.index(min(msg.ranges))    
    self.nearest_point_angle = self.nearest_point_index * self.angle_increment + self.angle_min
		

  def read_laser(self):
    while not self.ctrl_c:

      self.nearest_point_x = self.nearest_point_range * math.cos(self.nearest_point_angle)
      self.nearest_point_y = self.nearest_point_range * math.sin(self.nearest_point_angle)
      try:
          (trans,rot) = self.listener.lookupTransform('/chassis', '/hokuyo_front', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
   
      self.nearest_point_to_chassis_x = self.nearest_point_x + trans[0]
      self.nearest_point_to_chassis_y = self.nearest_point_y + trans[1]
      self.nearest_point_to_chassis = math.sqrt(self.nearest_point_to_chassis_x ** 2 + self.nearest_point_to_chassis_y ** 2)
      
      if self.nearest_point_to_chassis<5 and self.nearest_point_to_chassis>=2:
        print ("Warning!! Must be slow!! Distance is %.2f." % self.nearest_point_to_chassis) 
      elif self.nearest_point_to_chassis<2:
        print ("Dangerous!!! Must stop!!! Distance is %.2f." % self.nearest_point_to_chassis)
      else:
        print ("No Dangerous. Distance is %.2f." % self.nearest_point_to_chassis) 

      time.sleep(1)
      

  def shutdownhook(self):
    self.ctrl_c=True

if __name__=='__main__':
  rospy.init_node('demonstrator_preprocessing', anonymous=True)
  demonstrator_object=Demonstrator()

  try:
    demonstrator_object.read_laser()

  except rospy.ROSInterruptException:
    pass
