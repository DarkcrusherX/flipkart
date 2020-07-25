import sys
import copy
import rospy
import random
import math  
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from armf import armtakeoff

rospy.init_node('navigation',anonymous=True)

def main() :
    while True:
        local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 1
        pose.pose.position.z = 3.27

        local_pos_pub.publish(pose)  


if __name__ == '__main__':
     try:
        take = armtakeoff()
        take.arm()
        take.takeoff()
        main()

     except rospy.ROSInterruptException:
            pass


