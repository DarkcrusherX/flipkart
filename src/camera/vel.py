import sys
import copy
import rospy
import random
import math  
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
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
        publish_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=10)

        vel = TwistStamped()
        # pose.pose.position.x = 0.7

        global i
        t0=rospy.Time.now().to_sec()                                                   # To just wander left and ri8 to get a detection
        while rospy.Time.now().to_sec()-t0 < 2:                                        # its enough i think 
            if i%2 == 0:
                # pose.pose.position.y = 1
                if current_position.pose.position.z > 3.27:
                    error = current_position.pose.position.z-3.27
                    vel.twist.linear.z = -error
                elif current_position.pose.position.z < 3.27:
                    error = current_position.pose.position.z-3.27
                    vel.twist.linear.z = -error
                if current_position.pose.position.y < 2:
                    vel.twist.linear.y = 0.5
                publish_vel.publish(vel)

            else:
                # pose.pose.position.y = -1
                if current_position.pose.position.z > 3.27:
                    error = current_position.pose.position.z-3.27
                    vel.twist.linear.z = -error
                elif current_position.pose.position.z < 3.27:
                    error = current_position.pose.position.z-3.27
                    vel.twist.linear.z = -error
                if current_position.pose.position.y > -2:
                    vel.twist.linear.y = -0.5
                publish_vel.publish(vel)

        vel.twist.linear.y = 0
        publish_vel.publish(vel)            
            # pose.pose.position.z = 3.27  
            # pose.pose.orientation.w =1
            # local_pos_pub.publish(pose) 
        i = i+1



if __name__ == '__main__':
     try:
        take = armtakeoff()
        take.arm()
        take.takeoff()
        main()

     except rospy.ROSInterruptException:
            pass

