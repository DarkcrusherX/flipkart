import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from armf import armtakeoff

msg = PoseStamped()

rospy.init_node('points',anonymous=True)

setpoint = PoseStamped()
setpoint.pose.position.z=10
setpoint.pose.position.y=10
setpoint.pose.orientation.w=1
setpoint.header.frame_id = 'map'
setpoint.header.stamp= rospy.Time.now()

take = armtakeoff()
take.arm()
take.takeoff()

while True:
    setpoint_pub = rospy.Publisher('/command/pose', PoseStamped,queue_size=10)
    setpoint_pub.publish(setpoint)
    print(setpoint)





