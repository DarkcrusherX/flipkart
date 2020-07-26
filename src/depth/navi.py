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
oldbbox = detection()
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
    global bbox
    global oldbbox
    while True:
        setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
        rospy.Subscriber('cvmsg',detection,cvcallback)
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback_pos)
        # bbox.y = bbox.y - 1.5*bbox.breadth/3.5                                          Just in case if whole hoop is yellow
        oldbbox = bbox
        if bbox.length !=0 and bbox.breadth != 0:
            navigation()
        else:
            global i
            t0=rospy.Time.now().to_sec()                                                   # To just wander left and ri8 to get a detection
            while rospy.Time.now().to_sec()-t0 < 2:                                        # its enough i think 
                if i%2 == 0:
                    current_position.pose.position.y = 1
                else:
                    current_position.pose.position.y = -1

                if bbox.length !=0 and bbox.breadth != 0:
                    navigation()

                print("no proper detection : {}".format(bbox.y-240))
                current_position.pose.position.z = 3.4
                current_position.pose.orientation.w =1
                setpoint_pub.publish(current_position)
            i = i+1

def navigation():
    publish_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=10)
    
    while bbox.x > xpixel/2 + 2 or bbox.x < xpixel/2 - 2 :                              # Depends on the distance actually, if large distance then error of 1 even not sufficient.

        error = -bbox.x + xpixel/2
        vel.twist.linear.y = 0.003*error                                                # need to readjust , this also very much determines but greatly gets affected bcz of pixel scaling
        publish_vel.publish(vel)
        print("alligning error: {}".format(error))

    vel.twist.linear.x = 1                                                               # need to change velocity according to tolerable error

    distance = xpixel**2 /(1000*math.tan(fovx/2)*bbox.length) + 0.3                      # Giving boost until it get pass 0.3m from the hoop
    oldbbox = bbox

    time = distance/vel.twist.linear.x
    t1=rospy.Time.now().to_sec()
    print("time required to go to 0.3m from hoop : {}".format(time))

    while rospy.Time.now().to_sec()-t1 < time:                                                          # for trial onli
        if bbox.length > oldbbox.length and bbox.breadth > oldbbox.length:
            rospy.Subscriber('cvmsg',detection,cvcallback)
            print("realligning : {}".format(error))
            errorx = -bbox.x + xpixel/2
            vel.twist.linear.y = 0.013*errorx                        # need to readjust this value , this is the most important value in this code.
            errory = -bbox.y + ypixel/2
            vel.twist.linear.z = 0.008*errory                        # need to readjust this value , height may be wrong bcz of some t265 turbulance
        publish_vel.publish(vel)
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0

    vel.twist.linear.x = 0

if __name__ == '__main__':
     try:
        take = armtakeoff()
        take.arm()
        take.takeoff()
        cvfunction()


     except rospy.ROSInterruptException:
            pass


