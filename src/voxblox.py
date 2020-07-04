
import sys
import copy
import rospy
from mav_msgs.msg import RollPitchYawrateThrust
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import tf_conversions 

from armf import armtakeoff

rospy.init_node('voxboxTOmavros',anonymous=True)

msg = RollPitchYawrateThrust()
setpoint = PoseStamped()
setpoint.pose.position.x = 10
setpoint.pose.position.y = 10
setpoint.pose.position.z = 20
setpoint.header.frame_id = 'map'

setpoint.header.stamp= rospy.Time.now()


def callback(data):

    global msg

    msg = data

    main1()


def main1():
    global setpoint
    # thrust_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust',Thrust,queue_size=10)
    # thrust = Thrust()
    
    attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=10)
    point_pub = rospy.Publisher('/waypoint',PoseStamped,queue_size=10)
    attitude = AttitudeTarget()
    attitude.type_mask = 3
    attitude.header = msg.header 
    # attitude.header.frame_id = 'FRAME_LOCAL_NED'
    # quaternion = tf.transformations.quaternion_from_euler(msg.roll,msg.pitch, 0)
    # attitude.orientation.x = quaternion[0]
    # attitude.orientation.y = quaternion[1]
    # attitude.orientation.z = quaternion[2]
    # attitude.orientation.w = quaternion[3]
    attitude.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(msg.roll, msg.pitch, 0))

    attitude.body_rate.z = msg.yaw_rate
    t = msg.thrust.z/15
    if t>1:
        t=1
    elif t<-1:
        t=-1
    attitude.thrust = (t+1)/2
    attitude_pub.publish(attitude)
    point_pub.publish(setpoint)



if __name__ == '__main__':
     try:
            armclass = armtakeoff()

            rospy.Subscriber('/command/roll_pitch_yawrate_thrust',RollPitchYawrateThrust,callback)

            armclass.arm()

            armclass.takeoff()

            rospy.spin()

     except rospy.ROSInterruptException:
            pass




