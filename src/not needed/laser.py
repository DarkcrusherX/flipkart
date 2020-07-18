import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node('lidar_example', anonymous=True)

def lidar_callback(lidar_msg):
  # rospy.loginfo(lidar_msg.header)
  print(len(lidar_msg.ranges))
  j=0
  for i in range(len(lidar_msg.ranges)):
    if lidar_msg.ranges[i] <10:
      print(lidar_msg.ranges[i])
      j=j+1
  print(j)

sub_lidar = rospy.Subscriber("/laser/scan", LaserScan, lidar_callback)

rospy.spin()