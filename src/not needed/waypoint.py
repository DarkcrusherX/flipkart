import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from armf import armtakeoff

msg = PoseStamped()

rospy.init_node('waypoint',anonymous=True)

setpoint = PoseStamped()
setpoint.pose.position.z=8
setpoint.pose.position.y=10
setpoint.pose.position.x=20
setpoint.pose.orientation.w=1
setpoint.header.frame_id = 'map'
setpoint.header.stamp= rospy.Time.now()

# take = armtakeoff()
# take.arm()
# take.takeoff()

while True:
    setpoint_pub = rospy.Publisher('waypoint', PoseStamped,queue_size=10)
    setpoint_pub.publish(setpoint)
    print(setpoint)





