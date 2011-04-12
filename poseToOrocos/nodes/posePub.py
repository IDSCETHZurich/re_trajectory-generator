#!/usr/bin/env python
import roslib; roslib.load_manifest('poseToOrocos')
import rospy 
from geometry_msgs.msg import Pose
def posePulisher():
	pub = rospy.Publisher('poseDsr', Pose)
	rospy.init_node('posePublisher')
	poseDsr = Pose()

	while not rospy.is_shutdown():
		poseDsr.position.x = 0.3
		poseDsr.position.y = 0.3
		poseDsr.position.z = 0.3

		poseDsr.orientation.x = 0.0
		poseDsr.orientation.y = 0.0
		poseDsr.orientation.z = 0.0
		poseDsr.orientation.w = 1.0

		pub.publish(poseDsr)
		rospy.sleep(20.0)

if __name__ == '__main__':
	try:
		posePulisher()
	except rospy.ROSInterruptException: pass
