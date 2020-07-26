from armf import armtakeoff
import rospy

rospy.init_node('do_move1', anonymous=True)

a = armtakeoff()

a.arm()
a.takeoff1()

