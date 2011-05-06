import roslib
roslib.load_manifest('re_kuka')
import rospy
from re_kuka.srv import ExecuteCartesianIKTrajectory
from geometry_msgs.msg import Pose
import time
import sys
import pdb
import tf

def pplist(list):
    return ' '.join(['%2.3f,'%x for x in list])

if __name__ == "__main__":
    rospy.init_node("currentStateLookupNode")
    tf_listener = tf.TransformListener()
    time.sleep(.5) #give the transform listener time to get some frames

    #check the current pose
    (trans, rot) = tf_listener.lookupTransform('/arm_0_link', '/door3', rospy.Time(0))
    print "current Cartesian pose: trans", pplist(trans), "rot", pplist(rot)


