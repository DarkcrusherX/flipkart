
import sys
import copy
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory
from armf import armtakeoff

msg = MultiDOFJointTrajectory()
current_position= PoseStamped()

rospy.init_node('points',anonymous=True)

setpoint = PoseStamped()
setpoint.pose.position.x = 100
setpoint.pose.position.y = 100
setpoint.pose.position.z = 20
setpoint.pose.orientation.w = 1
setpoint.header.frame_id = 'map'

setpoint.header.stamp= rospy.Time.now()
setpoint_pub = rospy.Publisher('/waypoint', PoseStamped,queue_size=10)


def callback(data):

    global msg

    msg = data
    # print(len(msg.points))
    print("call back received")

def callback_pos(pos):

    global current_position
    current_position = pos

def calc_dist(trajectory,current_position):
    x1= trajectory.pose.position.x
    y1 = trajectory.pose.position.y
    # z1 = trajectory.pose.position.z
    x2= current_position.pose.position.x
    y2= current_position.pose.position.y
    # z2= current_position.pose.position.z


    dist = (x1-x2)**2 + (y1-y2)**2
    return dist

rospy.Subscriber('/mav_local_planner/full_trajectory',MultiDOFJointTrajectory,callback)
def main(i):
    global msg
    # global setpoint
    # setpoint_pub = rospy.Publisher('/waypoint', PoseStamped,queue_size=10)
    # setpoint_pub.publish(setpoint)
    # rospy.Subscriber('/mav_local_planner/full_trajectory',MultiDOFJointTrajectory,callback)

    publish = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)

    trajectory = PoseStamped()

    trajectory.pose.position = msg.points[i].transforms[0].translation
    trajectory.pose.orientation = msg.points[i].transforms[0].rotation
    trajectory.pose.position.z = 3

    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback_pos)

    dist =  calc_dist(trajectory,current_position)
    print(" Dist new : {}".format(dist))
    print(i)
    # print(dist)
    rate = rospy.Rate(10)
    while dist > 0.0005:

        rate.sleep()
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback_pos)

        dist =  calc_dist(trajectory,current_position)    
        trajectory.pose.position = msg.points[i].transforms[0].translation
        trajectory.pose.orientation = msg.points[i].transforms[0].rotation
        print("trajectory")
        print(i)
        print(" Dist : {}".format(dist))
        publish.publish(trajectory)



    if dist <0.0005 and i<(len(msg.points)-1):
        i = i+1
        main(i)
    elif dist <0.0005 and i>=len(msg.points):
        i=0
        msg =0
        while msg==0:
            rospy.Subscriber('/mav_local_planner/full_trajectory',MultiDOFJointTrajectory,callback)
            print("no hope")
        print("msg not zero")
        print(msg)
        main(i)


if __name__ == '__main__':
     try:
            armclass = armtakeoff()
            # global setpoint
            print(setpoint)
            setpoint_pub = rospy.Publisher('/waypoint', PoseStamped,queue_size=10)
            setpoint_pub.publish(setpoint)

            # rospy.sleep(1)

            rospy.Subscriber('/mav_local_planner/full_trajectory',MultiDOFJointTrajectory,callback)

            while msg==0:
                rospy.Subscriber('/mav_local_planner/full_trajectory',MultiDOFJointTrajectory,callback)
                print("no hope")
            print("msg not zero")
            print(msg)

            armclass.arm()
            armclass.takeoff()

            main(0)

     except rospy.ROSInterruptException:
            pass




