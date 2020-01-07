#!/usr/bin/env python
"""
Created on Sun Dec  1 17:52:30 2019

note:
    The code is using OpenCV2, make sure to remove OpenCV2 from cmakelist and xml if running on Kinetic.
    Implemented image projection, replace K with camera intrinsic parameter.
    P controller is used to align camera center with target.
"""
from __future__ import print_function
from geometry_msgs.msg import Twist
import roslib
roslib.load_manifest('vision_control')
import sys
import rospy
import cv2
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import acos


class k_control:
  
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    global state 
    state = 0
    rate = rospy.Rate(100)
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    vel_msg = Twist()
    lower_red = np.array([0,100,150])
    upper_red = np.array([10,255,255])
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            cv2.circle(cv_image, (int(x), int(y)), int(radius),(255,0, 0), 2)
            cv2.circle(cv_image, center, 5, (0, 255, 255), -1)
            (x,y) = center
            K = np.array([[530.4669406576809, 0.0, 320.5,], [0.0, 530.4669406576809, 240.5], [0.0, 0.0, 1.0]])
            ray = np.matmul( np.linalg.inv(K), np.array([[x],[y],[1]]))
            project_ray = np.array([ray[0][0], 0, ray[2][0]])
            z = np.array([0,0,1])
            nor_ray = project_ray/np.linalg.norm(project_ray)
            theta = acos(np.dot(nor_ray,z))
            if theta > 0.1:
                if ray[0][0] > 0:
                    #turn left
                    vel_msg.angular.z = -theta
                else:
                    #turn right
                    vel_msg.angular.z = theta
            else:
                vel_msg.angular.z = 0
                if radius < 70:
                    vel_msg.linear.x = 0.3
                else:
                    vel_msg.linear.x = 0
            
#==============================================================================
#             if state == 0:
#                 if theta > 0.005:
#                     if ray[0][0] > 0:
#                         #turn left
#                         vel_msg.angular.z = -1.5*theta
#                     else:
#                         #turn right
#                         vel_msg.angular.z = 1.5*theta
#                 if theta <= 0.006:
#                     state = 1
#                         
#             if state == 1:
#                 if theta < 0.01:
#                     vel_msg.angular.z = 0
#                     if radius < 70:
#                         vel_msg.linear.x = 0.3
#                     else:
#                         vel_msg.linear.x = 0
#                 else:
#                     vel_msg.linear.x = 0
#                     state = 0
#==============================================================================
              
        else:
            vel_msg.angular.z = -1
    else:
        vel_msg.angular.z = -1
 
    vel_msg.angular.x = 0    
    vel_msg.angular.y = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0    
    self.velocity_publisher.publish(vel_msg)
    rate.sleep()
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
 
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
    except CvBridgeError as e:
      print(e)
    
    

def main(args):
  ic = k_control()
  rospy.init_node('k_control', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)