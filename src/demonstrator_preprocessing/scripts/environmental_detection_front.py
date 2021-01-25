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




angle_min = -1.570796
angle_max = 3.1415926
sample = 720
angle_increment = (angle_max - angle_min) / sample  


def getPosition():
  i = 0
  j = 0

  listener = tf.TransformListener()
  
  position_x = list(range(non_inf_num))
  position_y = list(range(non_inf_num))
  
  
  while i < len(msg.ranges):     
    if i < len(msg.ranges)-2:

      if not math.isinf(msg.ranges[i]):
        point_range_0 = msg.ranges[i]
        point_range_1 = msg.ranges[i + 1]
        point_range_2 = msg.ranges[i + 2]
      
        if abs(point_range_0 - point_range_1) < 0.03 and abs(point_range_0 - point_range_2) < 0.03:
          point_range = msg.ranges[i]
          point_angle = i * angle_increment + angle_min
          point_x = point_range * math.cos(point_angle)
          point_y = point_range * math.sin(point_angle)
          count = 0
        
          while not count == 1:
            try:
              (trans,rot) = listener.lookupTransform('/world', '/hokuyo_front', rospy.Time(0))
              point_x_world = point_x + trans[0]
              point_y_world = point_y + trans[1]
          
              count = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              continue
          position_x[j] = point_x_world
          position_y[j] = point_y_world
          j += 1
          i += 3
        
        else:
          point_range = msg.ranges[i]
          point_angle = i * angle_increment + angle_min
          point_x = point_range * math.cos(point_angle)
          point_y = point_range * math.sin(point_angle)
          count = 0
        
          while not count == 1:
            try:
              (trans,rot) = listener.lookupTransform('/world', '/hokuyo_front', rospy.Time(0))
              point_x_world = point_x + trans[0]
              point_y_world = point_y + trans[1]
          
              count = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              continue


          position_x[j] = point_x_world
          position_y[j] = point_y_world
      
          j += 1
          i += 1
        

      else:
        i += 1
        continue
    else:
      if not math.isinf(msg.ranges[i]):
        point_range = msg.ranges[i]
        point_angle = i * angle_increment + angle_min
        point_x = point_range * math.cos(point_angle)
        point_y = point_range * math.sin(point_angle)
        count = 0
        
        while not count == 1:
          try:
            (trans,rot) = listener.lookupTransform('/world', '/hokuyo_front', rospy.Time(0))
            point_x_world = point_x + trans[0]
            point_y_world = point_y + trans[1]
          
            count = 1
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        position_x[j] = point_x_world
        position_y[j] = point_y_world
      
        j += 1
        i += 1
      else:
        i += 1
        continue






  position_x = [x for x in position_x if not isinstance(x, int)]
  position_y = [x for x in position_y if not isinstance(x, int)]
  
  return position_x, position_y

def setParam():
  global non_inf_num
  non_inf_num = len(msg.ranges) - msg.ranges.count(float("inf"))
  

  p_x, p_y = getPosition()
  ScanFilterChain=list(range(len(p_x)))
  
  for ind in range(len(p_x)):
    min_x = p_x[ind * 1] - 0.10
    min_y = p_y[ind * 1] - 0.10
    min_z = -5
    max_x = p_x[ind * 1] + 0.10
    max_y = p_y[ind * 1] + 0.10
    max_z = 5
    ScanFilterChain[ind] = {'type': 'laser_filters/LaserScanBoxFilter', 'params': {'min_x': min_x, 'min_y': min_y, 'min_z': min_z, 'invert': False, 'box_frame': 'world', 'max_z': max_z, 'max_x': max_x, 'max_y': max_y}, 'name': str(ind)}



  rospy.set_param('/laser_filter_front/scan_filter_chain', ScanFilterChain)

  print("Parameters for front lidar have been updated.")



### Main method
def main():
  rospy.init_node("environmental_detection_front")   
  
  msg_origin = rospy.wait_for_message("/laserscan/front", LaserScan)
  print("Positions have been recorded.")
  global msg 
  msg = msg_origin
  msg.ranges = [float("inf") if x > 6 else x for x in msg_origin.ranges]
  setParam()


### Entry point
if __name__ == '__main__':
#  rospy.loginfo("Ten seconds countdown......")
  print ("Ten seconds countdown......")
  for i in range(10):
    print (9 - i)
    time.sleep(1)
  main()
