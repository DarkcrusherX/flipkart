import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_conversions 
from geometry_msgs.msg import Quaternion
import numpy as np

rospy.init_node('lidar_localization', anonymous=True)

lidar = []
pose = Odometry()

def lidar_callback(lidar_msg):
    global lidar
    lidar = lidar_msg.ranges
    localization()

def pose_callback(pose_msg):
    global pose
    pose = pose_msg


def localization():
    global lidar
    global pose

    print(" ")
    right_max = 135
    right_min = 45
    left_max = 315
    left_min = 225
    left_lidar = lidar[left_min:left_max]

    right_lidar = lidar[right_min:right_max]


    dist_right = min(right_lidar)
    print("dist_right : {}".format(dist_right))
    angle_right = lidar.index(dist_right)
    dist_left = min(left_lidar)
    print("dist_left : {}".format(dist_left))
    angle_left = lidar.index(dist_left)


    error_right = (right_max + right_min)/2 - angle_right
    error_left = (left_max + left_min)/2 - angle_left 

    error = (error_right + error_left)/2 

    print(" deflection : {}".format(error))
    # print("left : {}".format(angle_left))
    # print("right : {}".format(angle_right))

    quaternion = Quaternion()
    quaternion = [pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w]

    roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(quaternion)
    print("Actual yaw : {} ".format(np.degrees(yaw)))



if __name__ == '__main__':
    try:

        sub_pose = rospy.Subscriber("/mavros/local_position/odom", Odometry, pose_callback)
        sub_lidar = rospy.Subscriber("/laser/scan", LaserScan, lidar_callback)

        rospy.spin()
            
    except rospy.ROSInterruptException:
        pass