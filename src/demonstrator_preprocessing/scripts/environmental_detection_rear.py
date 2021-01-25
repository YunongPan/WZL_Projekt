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
  j = 0
  listener = tf.TransformListener()
  
  position_x = list(range(non_inf_num))
  position_y = list(range(non_inf_num))
  
  
  for i in range(len(msg.ranges)):
    
    
    if not math.isinf(msg.ranges[i]):
      
      point_range = msg.ranges[i]
      point_angle = i * angle_increment + angle_min
      point_x = -(point_range * math.cos(point_angle))
      point_y = -(point_range * math.sin(point_angle))
      count = 0
      while not count == 1:
        try:
          (trans,rot) = listener.lookupTransform('/world', '/hokuyo_rear', rospy.Time(0))
          point_x_world = point_x + trans[0]
          point_y_world = point_y + trans[1]
          
          count = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue


      position_x[j] = point_x_world
      position_y[j] = point_y_world
      
      j += 1

  
  return position_x, position_y

def setParam():
  global non_inf_num
  non_inf_num = len(msg.ranges) - msg.ranges.count(float("inf"))
  ScanFilterChain=list(range(non_inf_num /1))

  p_x, p_y = getPosition()

  
  for ind in range(non_inf_num /1):
    min_x = p_x[ind * 1] - 0.1
    min_y = p_y[ind * 1] - 0.1
    min_z = -5
    max_x = p_x[ind * 1] + 0.1
    max_y = p_y[ind * 1] + 0.1
    max_z = 5
    ScanFilterChain[ind] = {'type': 'laser_filters/LaserScanBoxFilter', 'params': {'min_x': min_x, 'min_y': min_y, 'min_z': min_z, 'invert': False, 'box_frame': 'world', 'max_z': max_z, 'max_x': max_x, 'max_y': max_y}, 'name': str(ind)}



  rospy.set_param('/laser_filter_rear/scan_filter_chain', ScanFilterChain)
  print("Parameters for rear lidar have been updated.")



### Main method
def main():
  rospy.init_node("environmental_detection_rear")   
  
  msg_origin = rospy.wait_for_message("/laserscan/rear", LaserScan)
  print("Positions have been recorded.")
  global msg 
  msg = msg_origin
  msg.ranges = [float("inf") if x > 6 else x for x in msg_origin.ranges]
  setParam()
  

### Entry point
if __name__ == '__main__':
#  print ("Ten seconds countdown......")
  time.sleep(10)
  main()
