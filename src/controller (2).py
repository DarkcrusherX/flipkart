#!/usr/bin/env python3

# Module Imports
import math
import rospy
import smach
import mavros
import numpy as np
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import String
import smach_ros
import smach_viewer

## Global Variables
# FCU Connection State
current_state = State()
def state_cb(state):
  global current_state
  current_state = state

current_pose = PoseStamped()
def pos_cb(poseStamped):
  global current_pose
  current_pose = poseStamped

# Commaded Setpoint
setpoint = PositionTarget()
setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
setpoint.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE

# distance function
def DistToGoal():
  p1 = [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z]
  p2 = [setpoint.position.x, setpoint.position.y, setpoint.position.z]
  distance = math.sqrt( ((p1[0]-p2[0])**2) + ((p1[1]-p2[1])**2) + ((p1[2]-p2[2])**2) )
  return distance
  # return math.dist([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z],[setpoint.position.x, setpoint.position.y, setpoint.position.z])

# Hoops Lookup Table
hoops = [1, 0, 2, -1, -1.5, 0.5, -1.5, 1.5, -1.5, 1, -0.5, 2, 0, -1, 1]
counter = 0
y_off = -0.10
x_off = 0.05

## Publsiher and Subscriber Objects
state_sub = rospy.Subscriber("mavros/state", State, state_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pos_cb)
SM_pub = rospy.Publisher("grid/state", String, queue_size=10)
land_srv = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

## SMACH State Machine
class Takeoff(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                        outcomes=['GoToYScan'],
                        input_keys=['rate'],
                        output_keys=['rate'])

  def execute(self, userdata):
    SM_pub.publish(String("Takeoff"))

    global setpoint
    setpoint.position = Point(1+x_off, 0+y_off, 3)
    setpoint.velocity = Point(0, 0, 0)
    reachedFlag = False
    
    while not rospy.is_shutdown():
      local_pos_pub.publish(setpoint)
      
      if not reachedFlag:
        if DistToGoal() < 0.1:
          reachedFlag = True
          reachTime = rospy.get_rostime()

      if reachedFlag:
        if (rospy.get_rostime() - reachTime > rospy.Duration(1.0)):
          return 'GoToYScan'

      userdata.rate.sleep()

class Yscan(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                        outcomes=['Alignment'],
                        input_keys=['rate'],
                        output_keys=['rate'])

  def execute(self, userdata):
    SM_pub.publish(String("Yscan"))
    global setpoint
    setpoint.type_mask = setpoint.type_mask | PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    setpoint.position = Point(counter + 1 + x_off, hoops[counter] + y_off, 3)
    setpoint.velocity = Point(0, -0.5, 0)
    if (current_pose.pose.position.y < hoops[counter]):
      toReverse = True
    else :
      toReverse = False
    
    reachedFlag = False

    while not rospy.is_shutdown():
      local_pos_pub.publish(setpoint)

      if toReverse:
        if abs(current_pose.pose.position.y + 2 + y_off) < 0.1 :
          toReverse = False
          setpoint.velocity = Point(0, 0.5, 0)
      
      if not reachedFlag:
        if abs(current_pose.pose.position.y - hoops[counter]) < 0.1:
          reachedFlag = True
          setpoint.type_mask = setpoint.type_mask ^ PositionTarget.IGNORE_PX ^ PositionTarget.IGNORE_PY ^ PositionTarget.IGNORE_PZ
          setpoint.velocity = Point(0, 0, 0)
          reachTime = rospy.get_rostime()
      
      else :
        if (rospy.get_rostime() - reachTime > rospy.Duration(3.0)):
          return 'Alignment'
      
      userdata.rate.sleep()

class Boost(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                        outcomes=['GoToYScan','Land'],
                        input_keys=['rate'],
                        output_keys=['rate'])

  def execute(self, userdata):
    SM_pub.publish(String("Boost"))
    global setpoint
    global counter
    setpoint.type_mask = setpoint.type_mask | PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    setpoint.velocity = Point(0.5, 0, 0)
    setpoint.position = Point(counter+2+x_off, hoops[counter]+y_off, 3)
    reachedFlag = False

    while not rospy.is_shutdown():
      if not reachedFlag:
        if abs(current_pose.pose.position.x - counter-2-x_off) <0.1:
          reachedFlag = True
          setpoint.type_mask = setpoint.type_mask ^ PositionTarget.IGNORE_PX ^ PositionTarget.IGNORE_PY ^ PositionTarget.IGNORE_PZ
          if counter == 14:
            setpoint.position = Point(counter+2+x_off+2, 0+y_off, 3)
          setpoint.velocity = Point(0, 0, 0)
          counter=counter+1
          reachTime = rospy.get_rostime()
      
      if reachedFlag:
        if counter == 15:
          if (rospy.get_rostime() - reachTime > rospy.Duration(3.0)):
            return 'Land'
        elif (rospy.get_rostime() - reachTime > rospy.Duration(1.0)):
            return 'GoToYScan'
      
      
      local_pos_pub.publish(setpoint)
      userdata.rate.sleep()

## Main program
def main():
  rospy.init_node('controller')
  rospy.loginfo("Program Start")
  rate = rospy.Rate(40.0)
  
  # Wait for FCU to start
  while not current_state.connected:
    rate.sleep()
  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['Land'])
  sm.userdata.rate = rate

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'GoToYScan': 'YSCAN'})
    smach.StateMachine.add('YSCAN', Yscan(), transitions={'Alignment': 'BOOST'})
    smach.StateMachine.add('BOOST', Boost(), transitions={'GoToYScan': 'YSCAN'})

  sis = smach_ros.IntrospectionServer('Grid', sm, '/StateMachine')
  sis.start()
  
  # Execute SMACH plan
  sm.execute()
  land_srv.call()
  sis.stop()

if __name__ == '__main__':
    try:
      main()
    except rospy.ROSInterruptException:
      pass