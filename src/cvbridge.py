import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
rospy.init_node('opencv_example', anonymous=True)

def show_image(img):
  cv2.namedWindow("Image Window")
  # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  img = img*30/255      # i dont know some thing is wrong
  img = cv2.resize(img, (640, 480)) 
  cv2.imshow("Image Window", img)
  print(img.shape)
  cv2.waitKey(3)


def image_callback(img_msg):
#   rospy.loginfo(img_msg.header)
  cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
  show_image(cv_image)   

sub_image = rospy.Subscriber("/camera/depth/image_raw", Image, image_callback)

rospy.spin()