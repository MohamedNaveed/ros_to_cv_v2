#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('ros_to_cv_v2')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_depth:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_depth",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

    (rows,cols) = cv_image.shape
    print("rows:", rows, "cols:", cols)
    #if cols > 60 and rows > 60 :
     # cv2.circle(cv_image, (300,500), 25, (0,255,0))

    for a in range(rows):
        print("depth", cv_image[a][100])
    cv2.imshow('depth image', cv_image)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "passthrough"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_depth()
  rospy.init_node('image_depth', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
