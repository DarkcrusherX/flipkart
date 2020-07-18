import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from flipkart.msg import detection
import numpy as np

dbridge = CvBridge()
rospy.init_node('opencvdepth', anonymous=True)
bbox = detection()

def show_image(frame):
  frame= frame*10                                                         # for representation
  cv.namedWindow("depth Image Window",cv.WINDOW_NORMAL)
  cv.namedWindow("bounding Image Window",cv.WINDOW_NORMAL)
  cv.namedWindow("intermediate Image Window",cv.WINDOW_NORMAL)
  cv.imshow("depth Image Window",frame)
  mask = cv.inRange(frame, np.array(0), np.array(10))
  mask_Open = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((10, 10)))
  mask_Close = cv.morphologyEx(mask_Open, cv.MORPH_CLOSE, np.ones((20, 20))) #to reduce noise and get smooth image
  mask_Perfect = mask_Close
  cv.imshow("bounding Image Window",mask_Perfect)
  conts, h = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) # detects contours
  for c in conts:  #creates bounding boxes around the detected hoop
    # Find the index of the largest contour
    areas = [cv.contourArea(c) for c in conts] 
    max_index = np.argmax(areas)
    cnt=conts[max_index]
    x, y, w, h = cv.boundingRect(cnt)
    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
    cv.circle(frame, (x + int(w*0.5), y + int(h*0.5)), 4, (0,0,255), -1) #locates the center of bounding box
    # print(x + int(w * 0.5), y + int(h * 0.5))  #center of the bounding box
  cv.imshow("bounding Image Window",frame)
  cv.waitKey(3)


def image_callback(img_msg):
  cv_image = dbridge.imgmsg_to_cv2(img_msg,"8UC1")
  show_image(cv_image)   

sub_image = rospy.Subscriber("/camera/depth/image_raw", Image, image_callback)

rospy.spin()
