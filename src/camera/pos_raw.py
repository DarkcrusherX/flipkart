import sys
import copy
import rospy
import random
import math  
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
from armf import armtakeoff

rospy.init_node('navigation',anonymous=True)
i=0
current_position = PoseStamped()

def callback_pos(pos):

    global current_position
    current_position = pos

def main() :
    while True:
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback_pos)
        local_pos_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        pose = PositionTarget()
        pose.position.x = 0.9

        pose.velocity.z = 0
        pose.acceleration_or_force.z = 0
        global i
        t0=rospy.Time.now().to_sec()                                                   # To just wander left and ri8 to get a detection
        while rospy.Time.now().to_sec()-t0 < 2:                                        # its enough i think 
            # if i%2 == 0:
            #     pose.position.y = 1
            # else:
            #     pose.position.y = -1
            pose.position.y = 0.2
            pose.position.z = 3
            local_pos_pub.publish(pose) 
        i = i+1



if __name__ == '__main__':
     try:
        take = armtakeoff()
        take.arm()
        take.takeoff()
        main()

     except rospy.ROSInterruptException:
            pass


