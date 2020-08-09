#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

rospy.init_node('realsense', anonymous=True)

def pos_callback(pos_msg):
    odom = pos_msg
    
    pos1 = PoseWithCovarianceStamped()
    pos1.header = odom.header
    pos1.pose = odom.pose
    pos1.pose

    # pos = PoseStamped()
    # pos.header = odom.header
    # pos.pose = odom.pose.pose

    pub_pos_cov.publish(pos1)
    # pub_pos.publish(pos)

    # # pos.header.frame_id = "map"
    # pos.pose.position.x = round(position.position.x,12)
    # pos.pose.position.y = round(position.position.y,12)
    # pos.pose.position.z = round(position.position.z,12)
    # pos.pose.orientation.x = round(position.orientation.x,12)
    # pos.pose.orientation.y = round(position.orientation.y,12)
    # pos.pose.orientation.z = round(position.orientation.z,12)
    # pos.pose.orientation.w = round(position.orientation.w,12)
    #  = odom.pose.covariance
    # print(pos)
    # pub_pos.publish(pos)


# sub_pos = rospy.Subscriber("/gazebo/model_states", ModelStates, pos_callback)

sub_pos = rospy.Subscriber("/ground_truth/state", Odometry, pos_callback)

# pub_pos = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=30)
pub_pos_cov = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=30)



rospy.spin()