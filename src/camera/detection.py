import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
# from geometry_msgs.msg import PoseStamped
from flipkart.msg import detection
import numpy as np

bridge = CvBridge()
rospy.init_node('opencv_example', anonymous=True)

mid_pub = rospy.Publisher('cvmsg', detection,queue_size=10)
midpoint = detection()

def show_image(frame):
  cv.namedWindow("Image Window")
  lower = np.array([20, 100, 100])  #hsv limits vary based on hoops yellow
  upper = np.array([30, 255, 255])

  frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
  mask = cv.inRange(frame_HSV, lower, upper)
  mask_Open = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((10, 10)))
  mask_Close = cv.morphologyEx(mask_Open, cv.MORPH_CLOSE, np.ones((20, 20))) #to reduce noise and get smooth image
  mask_Perfect = mask_Close
  _,conts, h = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) # detects contours
  
  for c in conts:  #creates bounding boxes around the detected hoop
    # Find the index of the largest contour
    areas = [cv.contourArea(c) for c in conts] 
    max_index = np.argmax(areas)
    cnt=conts[max_index]
    x, y, w, h = cv.boundingRect(cnt)
    cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv.circle(frame, (x + int(w*0.5), y + int(h*0.5)), 4, (0,0,255), -1) #locates the center of bounding box
    print(x + int(w * 0.5), y + int(h * 0.5))  #center of the bounding box
    print("length : {}".format(w))
    print("breadth : {}".format(h))
    # print(h/w)
    # if 1< h/w <5:                                #raandom values      onli if whole hoop is yellow , else peace

    #   midpoint.x = x + int(w * 0.5)
    #   midpoint.y = y + int(h * 0.5)
    #   midpoint.length = w
    #   midpoint.breadth = h
    # else:
    #   midpoint.x = 0
    #   midpoint.y = 0
    #   midpoint.length = 0
    #   midpoint.breadth = 0     
    midpoint.x = x + int(w * 0.5)
    midpoint.y = y + int(h * 0.5)
    midpoint.length = w
    midpoint.breadth = h
  mid_pub.publish(midpoint)
  # img = cv.cvtColor(frame, cv.COLOR_BGR2RGB)  
  cv.imshow("Image Window",frame)
  cv.waitKey(3)


def image_callback(img_msg):
  # rospy.loginfo(img_msg.header)
  cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
  cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)  
  show_image(cv_image)   

sub_image = rospy.Subscriber("/iris/usb_cam/image_raw", Image, image_callback)

rospy.spin()
