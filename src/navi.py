import sys
import copy
import rospy
import random
import math  
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from flipkart.msg import detection
from armf import armtakeoff

rospy.init_node('navigation',anonymous=True)

xpixel = 640
ypixel = 480
fovx = 1.047

bbox = detection()
setpoint = PoseStamped()
current_position = PoseStamped()
vel = TwistStamped()
i=0

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
        # bbox.y = bbox.y - 1.5*bbox.breadth/3.5         Just in case if whole hoop is yellow
        if ypixel/2 - 15 < bbox.y < ypixel/2 + 15:
            navigation()
        else:
            global i
            t0=rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec()-t0 < 2:                                   # its enough i think
                if i%2 == 0:
                    current_position.pose.position.y = 1
                else:
                    current_position.pose.position.y = -1

                if ypixel/2 - 15 < bbox.y < ypixel/2 + 15:
                    navigation()

                print("no proper detection : {}".format(bbox.y-240))
                current_position.pose.position.z = 3.4
                current_position.pose.orientation.w =1
                setpoint_pub.publish(current_position)
            i = i+1

def navigation():
    publish_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=10)

    while bbox.x > xpixel/2 + 1 or bbox.x < xpixel/2 - 1 : 

        error = -bbox.x + xpixel/2
        vel.twist.linear.y = 0.003*error
        publish_vel.publish(vel)
        print("alligning error: {}".format(error))

    vel.twist.linear.x = 1

    distance = xpixel**2 /(1000*math.tan(fovx/2)*bbox.length)

    # distance = (bbox.length*xpixel)/(1000*math.tan(fovx/2)) + 0.5

    time = distance/vel.twist.linear.x
    t1=rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec()-t1 < time:                             # for trial onli
        publish_vel.publish(vel)
        print("time required to go to 0.5m from hoop : {}".format(time))
    vel.twist.linear.x = 0

if __name__ == '__main__':
     try:
        take = armtakeoff()
        take.arm()
        take.takeoff()
        cvfunction()


     except rospy.ROSInterruptException:
            pass


