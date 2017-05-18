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



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    #print("rows:", rows, "cols:", cols)
    """if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (300,500), 25, (0,255,0))"""

    pts1 = np.float32([[300,300],[500,300],[500,540],[300,500]])
    pts2 = np.float32([[0,0],[960,0],[960,540],[0,540]])

    M = cv2.getPerspectiveTransform(pts1,pts2)
    img = cv2.warpPerspective(cv_image,M,(960,540))
    cv2.imshow("Perspective",img)

    imageGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #Convert BGR to Gray
    ret, thresh=cv2.threshold(imageGray,135,255,cv2.THRESH_BINARY)
    cv2.imshow("Thresh", thresh)
    #contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, 2)

    #DRAW contours
    cv2.drawContours(img, contours, -1, (0,255,0), 3)

    cv2.imshow("Gray", imageGray)
    cv2.waitKey(3)

    print("number of contours:", len(contours))

    for i in range(len(contours)):
        c=contours[i]
        area=cv2.contourArea(c)

        if area>7000:
            M=cv2.moments(c)
            if M["m00"] == 0:
                continue

            cx=int(M['m10']/M['m00'])
            cy=int(M['m01']/M['m00'])
            cv2.circle(img, (cx,cy), 1, (0,0,255))

            print(i, "=", "cx:" , cx , "cy:", cy , "area" , area)
    cv2.imshow("Image window", img)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
