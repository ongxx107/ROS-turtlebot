#!/usr/bin/env python
"""
Created on Sun Dec  1 17:52:30 2019

note:
    The code is using OpenCV2, make sure to remove OpenCV2 from cmakelist and xml if running on Kinetic.
"""
from __future__ import print_function
from geometry_msgs.msg import Twist
import roslib
roslib.load_manifest('vision_control')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils

class pixel_control:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
    vel_msg = Twist()
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])
    frame = imutils.resize(cv_image, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
            cv2.circle(frame, (int(x), int(y)), int(radius),(255,0, 0), 2)
            cv2.circle(frame, center, 5, (0, 255, 255), -1)
            (x,y) = center
            bias = x-300
            if bias < 0:
                vel_msg.angular.z = 0.1
            if bias > 0 :
                vel_msg.angular.z = -0.1
            if abs(bias) < 10:
                vel_msg.angular.z = 0
                if radius < 70:
                    vel_msg.linear.x = 0.3
                else:
                    vel_msg.linear.x = 0
        else:
            vel_msg.angular.z = -0.1
    else:
        vel_msg.angular.z = -0.1
    
    
    cv_image = frame
    
    
    vel_msg.angular.x = 0    
    vel_msg.angular.y = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0    
    self.velocity_publisher.publish(vel_msg)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
 
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
    except CvBridgeError as e:
      print(e)
    
    

def main(args):
  ic = pixel_control()
  rospy.init_node('pixel_control', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)