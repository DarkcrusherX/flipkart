import sys
import copy
import rospy
import random
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from flipkart.msg import detection
from armf import armtakeoff

rospy.init_node('navigation',anonymous=True)

xpixel = 640
ypixel = 480
bbox = detection()
setpoint = PoseStamped()
current_position = PoseStamped()
vel = TwistStamped()

def cvcallback(cv_msg):

    global bbox
    bbox = cv_msg

def callback_pos(pos):

    global current_position
    current_position = pos

def cvfunction():
    while True:
        setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
        rospy.Subscriber('cvmsg',detection,cvcallback)
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback_pos)
        bbox.y = bbox.y - 1.5*bbox.breadth/3.5
        if ypixel/2 - 30 < bbox.y < ypixel/2 + 30:
            navigation()
        else:
            i=0
            t0=rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec()-t0 < 2:
                if i%2 == 0:
                    current_position.pose.position.y = 1
                else:
                    current_position.pose.position.y = -1

            print("no proper detection : {}".format(bbox.y-240))
            current_position.pose.position.z = 3.5
            setpoint_pub.publish(current_position)
            i = i+1

def navigation():
    publish_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=10)

    while bbox.x > xpixel/2 +30 and bbox.y < xpixel/2 -30 : 
        error = bbox.x - xpixel/2
        vel.twist.linear.y = 0.001*error
        publish_vel.publish(vel)
        print("alligning")
    vel.twist.linear.x = 2
    while True:
        publish_vel.publish(vel)


if __name__ == '__main__':
     try:
        take = armtakeoff()
        take.arm()
        take.takeoff()
        cvfunction()


     except rospy.ROSInterruptException:
            pass


