import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from flipkart.msg import detection
import numpy as np

dbridge = CvBridge()
rospy.init_node('opencvdepth', anonymous=True)
bbox = detection()

def cvcallback(cv_msg):

  global bbox
  bbox = cv_msg
  # print(bbox)

def show_image(frame):
  frame= frame*10                                                         # for representation
  cv.waitKey(1)
  cv.namedWindow("depth Image Window",cv.WINDOW_NORMAL)
  rospy.Subscriber('cvmsg',detection, cvcallback)

  x = bbox.x
  y = bbox.y
  h = bbox.breadth
  w = bbox.length

  value = frame[int(x)][int(y)]
  print(len(frame))
  cv.rectangle(frame,(int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),(255, 255, 255), 2,1)

  print("DEpth Value: {}".format(value))

  cv.imshow("depth Image Window",frame)
  cv.waitKey(6)


def image_callback(img_msg):
  cv_image = dbridge.imgmsg_to_cv2(img_msg,"8UC1")
  show_image(cv_image)   

sub_image = rospy.Subscriber("/camera/depth/image_raw", Image, image_callback)

rospy.spin()
