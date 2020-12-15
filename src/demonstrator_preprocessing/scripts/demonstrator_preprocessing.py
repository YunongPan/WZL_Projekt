#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('demonstrator_preprocessing')
import time
import math
import tf
import geometry_msgs.msg
import message_filters
from sensor_msgs.msg import LaserScan

class Demonstrator():
  def __init__(self):
    self.laser_front_sub = message_filters.Subscriber("laserscan/front", LaserScan)
    self.laser_rear_sub = message_filters.Subscriber("laserscan/rear", LaserScan)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.laser_front_sub, self.laser_rear_sub], queue_size = 10, slop=0.5)
    self.ts.registerCallback(self.scan_callback)

    self.listener = tf.TransformListener()

    self.angle_min = -1.570796
    self.angle_max = 3.1415926
    self.sample = 720
    self.angle_increment = (self.angle_max - self.angle_min) / self.sample   
 
    self.nearest_point_range_front = 0.0
    self.nearest_point_range_rear = 0.0
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
	
  def scan_callback(self, laser_front, laser_rear):
    self.nearest_point_range_front = min(laser_front.ranges)
    self.nearest_point_range_rear = min(laser_rear.ranges)
    if self.nearest_point_range_front < self.nearest_point_range_rear:
      self.nearest_point_index = laser_front.ranges.index(min(laser_front.ranges))    
      self.nearest_point_angle = self.nearest_point_index * self.angle_increment + self.angle_min
      self.nearest_point_x = self.nearest_point_range_front * math.cos(self.nearest_point_angle)
      self.nearest_point_y = self.nearest_point_range_front * math.sin(self.nearest_point_angle)
    else:
      self.nearest_point_index = laser_rear.ranges.index(min(laser_rear.ranges))    
      self.nearest_point_angle = self.nearest_point_index * self.angle_increment + self.angle_min
      self.nearest_point_x = self.nearest_point_range_rear * math.cos(self.nearest_point_angle)
      self.nearest_point_y = self.nearest_point_range_rear * math.sin(self.nearest_point_angle)

  def read_laser(self):
    while not self.ctrl_c:


      try:
        if self.nearest_point_range_front < self.nearest_point_range_front:
          (trans,rot) = self.listener.lookupTransform('/chassis', '/hokuyo_front', rospy.Time(0))
          self.nearest_point_to_chassis_x = self.nearest_point_x + trans[0]
          self.nearest_point_to_chassis_y = self.nearest_point_y + trans[1]
          self.nearest_point_to_chassis = math.sqrt(self.nearest_point_to_chassis_x ** 2 + self.nearest_point_to_chassis_y ** 2)

        else:
          (trans,rot) = self.listener.lookupTransform('/chassis', '/hokuyo_rear', rospy.Time(0))

      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
   
      self.nearest_point_to_chassis_x = self.nearest_point_x - trans[0]
      self.nearest_point_to_chassis_y = self.nearest_point_y - trans[1]
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
  rospy.init_node('demonstrator_preprocessing')
  demonstrator_object=Demonstrator()

  try:
    demonstrator_object.read_laser()

  except rospy.ROSInterruptException:
    pass
