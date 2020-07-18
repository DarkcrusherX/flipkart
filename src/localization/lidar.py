import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_conversions 
from geometry_msgs.msg import Quaternion
import numpy as np
import math

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

    quaternion = Quaternion()
    quaternion = [pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w]

    roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(quaternion)


    print(" ")
    right_max = 120
    right_min = 60
    left_max = 300
    left_min = 240 

    left_angles = []
    right_angles = []    

    left_angles =  np.arange(left_min , left_max , 1)
    right_angles = np.arange(right_min , right_max , 1)

    left_lidar = lidar[left_min:left_max]
    right_lidar = lidar[right_min:right_max]
    right_lidar = np.array(right_lidar)
    right_lidar = np.clip(right_lidar,0,15)
    left_lidar = np.array(left_lidar)
    left_lidar = np.clip(left_lidar,0,15)

    xl = []
    yl = []
    xr = []
    yr = []

    xl = left_lidar*np.cos(left_angles*np.pi/180)
    yl = left_lidar*np.sin(left_angles*np.pi/180)
    xr = right_lidar*np.cos(right_angles*np.pi/180)
    yr = right_lidar*np.sin(right_angles*np.pi/180)

    slxy = np.dot(xl,yl.T) - np.average(xl)*np.average(yl)*(left_max-left_min)
    slxx = np.dot(xl,xl.T) - np.average(xl)**2*(left_max-left_min)

    m_l = slxy/slxx
    c_l = np.average(yl) - m_l*np.average(xl)

    srxy = np.dot(xr,yr.T) - np.average(xr)*np.average(yr)*(right_max-right_min)
    srxx = np.dot(xr,xr.T) - np.average(xr)**2*(right_max-right_min)

    m_r = srxy/srxx
    c_r = np.average(yr) - m_r*np.average(xr)

    pl = c_l/math.sqrt(1+m_l**2)
    al = -math.tanh(m_l)*180/math.pi

    pr = c_r/math.sqrt(1+m_r**2)
    ar = -math.tanh(m_r)*180/math.pi

    p = pr+pl/2
    a = ar+al/2

    print("mavros yaw : {}".format(np.degrees(yaw)))
    print("lidar yaw : {}".format(a))
    # print(m_l)
    # print(c_l)


if __name__ == '__main__':
    try:

        sub_pose = rospy.Subscriber("/mavros/local_position/odom", Odometry, pose_callback)
        sub_lidar = rospy.Subscriber("/laser/scan", LaserScan, lidar_callback)

        rospy.spin()
            
    except rospy.ROSInterruptException:
        pass